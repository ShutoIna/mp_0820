// Copyright 2019 The MediaPipe Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#import "ViewController.h"

#import "mediapipe/objc/MPPGraph.h"
#import "mediapipe/objc/MPPCameraInputSource.h"
#import "mediapipe/objc/MPPLayerRenderer.h"

#include "mediapipe/framework/formats/landmark.pb.h"

static NSString* const kGraphName = @"multi_hand_tracking_mobile_gpu";

static const char* kInputStream = "input_video";
static const char* kOutputStream = "output_video";
static const char* kLandmarksOutputStream = "multi_hand_landmarks";
static const char* kVideoQueueLabel = "com.google.mediapipe.example.videoQueue";

//static NSString *pth = @"~/Documents";

static NSString * pth01 = (NSString *)
[NSSearchPathForDirectoriesInDomains(NSDocumentDirectory,
                                     NSUserDomainMask,
                                     YES) lastObject];

//h=hand_indexです．0=左手, 1=右手
//i=indexです． 0-20の21点の座標
//x,y,z=21点のxyz座標
//t=time

//各情報を格納するpath及びcsvファイル
static NSString *pth_t = [pth01 stringByAppendingPathComponent:@"test_t.csv"];
static NSString *pth_h = [pth01 stringByAppendingPathComponent:@"test_h.csv"];
static NSString *pth_i = [pth01 stringByAppendingPathComponent:@"test_i.csv"];
static NSString *pth_x = [pth01 stringByAppendingPathComponent:@"test_x.csv"];
static NSString *pth_y = [pth01 stringByAppendingPathComponent:@"test_y.csv"];
static NSString *pth_z = [pth01 stringByAppendingPathComponent:@"test_z.csv"];

//csvに入れるための文字ファイル
static  NSMutableString* mstr_t = [[NSMutableString alloc] init];
static  NSMutableString* mstr_h = [[NSMutableString alloc] init];
static  NSMutableString* mstr_i = [[NSMutableString alloc] init];
static  NSMutableString* mstr_x = [[NSMutableString alloc] init];
static  NSMutableString* mstr_y = [[NSMutableString alloc] init];
static  NSMutableString* mstr_z = [[NSMutableString alloc] init];

 //static NSSavePanel *panel = [NSSavePanel savePanel];
// セーブ・パネルを取得
 // セーブ・パネルを表示

static NSString *filePath = @"/Users/inagakishuto/Desktop/test.csv";
static NSString *text = [NSString stringWithContentsOfFile:filePath encoding:NSUTF8StringEncoding error:nil];

@interface ViewController () <MPPGraphDelegate, MPPInputSourceDelegate>

// The MediaPipe graph currently in use. Initialized in viewDidLoad, started in viewWillAppear: and
// sent video frames on _videoQueue.
@property(nonatomic) MPPGraph* mediapipeGraph;

@end

@implementation ViewController {
  /// Handles camera access via AVCaptureSession library.
  MPPCameraInputSource* _cameraSource;

  /// Inform the user when camera is unavailable.
  IBOutlet UILabel* _noCameraLabel;
  /// Display the camera preview frames.
  IBOutlet UIView* _liveView;
  /// Render frames in a layer.
  MPPLayerRenderer* _renderer;

  /// Process camera frames on this queue.
  dispatch_queue_t _videoQueue;
}

#pragma mark - Cleanup methods

- (void)dealloc {
  self.mediapipeGraph.delegate = nil;
  [self.mediapipeGraph cancel];
  // Ignore errors since we're cleaning up.
  [self.mediapipeGraph closeAllInputStreamsWithError:nil];
  [self.mediapipeGraph waitUntilDoneWithError:nil];
}

#pragma mark - MediaPipe graph methods

+ (MPPGraph*)loadGraphFromResource:(NSString*)resource {
  // Load the graph config resource.
  NSError* configLoadError = nil;
  NSBundle* bundle = [NSBundle bundleForClass:[self class]];
  if (!resource || resource.length == 0) {
    return nil;
  }
  NSURL* graphURL = [bundle URLForResource:resource withExtension:@"binarypb"];
  NSData* data = [NSData dataWithContentsOfURL:graphURL options:0 error:&configLoadError];
  if (!data) {
    NSLog(@"Failed to load MediaPipe graph config: %@", configLoadError);
    return nil;
  }

  // Parse the graph config resource into mediapipe::CalculatorGraphConfig proto object.
  mediapipe::CalculatorGraphConfig config;
  config.ParseFromArray(data.bytes, data.length);

  // Crea    te MediaPipe graph with mediapipe::CalculatorGraphConfig proto object.
  MPPGraph* newGraph = [[MPPGraph alloc] initWithGraphConfig:config];
  [newGraph addFrameOutputStream:kOutputStream outputPacketType:MPPPacketTypePixelBuffer];
  [newGraph addFrameOutputStream:kLandmarksOutputStream outputPacketType:MPPPacketTypeRaw];
  return newGraph;
}

#pragma mark - UIViewController methods

- (void)viewDidLoad {
  [super viewDidLoad];

  _renderer = [[MPPLayerRenderer alloc] init];
  _renderer.layer.frame = _liveView.layer.bounds;
  [_liveView.layer addSublayer:_renderer.layer];
  _renderer.frameScaleMode = MPPFrameScaleModeFillAndCrop;
  // When using the front camera, mirror the input for a more natural look.
  _renderer.mirrored = NO;

  dispatch_queue_attr_t qosAttribute = dispatch_queue_attr_make_with_qos_class(
      DISPATCH_QUEUE_SERIAL, QOS_CLASS_USER_INTERACTIVE, /*relative_priority=*/0);
  _videoQueue = dispatch_queue_create(kVideoQueueLabel, qosAttribute);

  _cameraSource = [[MPPCameraInputSource alloc] init];
  [_cameraSource setDelegate:self queue:_videoQueue];
  _cameraSource.sessionPreset = AVCaptureSessionPresetHigh;
  _cameraSource.cameraPosition = AVCaptureDevicePositionBack;
  // The frame's native format is rotated with respect to the portrait orientation.
  _cameraSource.orientation = AVCaptureVideoOrientationPortrait;

  self.mediapipeGraph = [[self class] loadGraphFromResource:kGraphName];
  self.mediapipeGraph.delegate = self;
  // Set maxFramesInFlight to a small value to avoid memory contention for real-time processing.
  self.mediapipeGraph.maxFramesInFlight = 2;
}

// In this application, there is only one ViewController which has no navigation to other view
// controllers, and there is only one View with live display showing the result of running the
// MediaPipe graph on the live video feed. If more view controllers are needed later, the graph
// setup/teardown and camera start/stop logic should be updated appropriately in response to the
// appearance/disappearance of this ViewController, as viewWillAppear: can be invoked multiple times
// depending on the application navigation flow in that case.
- (void)viewWillAppear:(BOOL)animated {
  [super viewWillAppear:animated];

  [_cameraSource requestCameraAccessWithCompletionHandler:^void(BOOL granted) {
    if (granted) {
      [self startGraphAndCamera];
      dispatch_async(dispatch_get_main_queue(), ^{
        _noCameraLabel.hidden = YES;
      });
    }
  }];
}

- (void)startGraphAndCamera {
  // Start running self.mediapipeGraph.
  NSError* error;
  if (![self.mediapipeGraph startWithError:&error]) {
    NSLog(@"Failed to start graph: %@", error);
  }

  // Start fetching frames from the camera.
  dispatch_async(_videoQueue, ^{
    [_cameraSource start];
  });
}

#pragma mark - MPPGraphDelegate methods

// Receives CVPixelBufferRef from the MediaPipe graph. Invoked on a MediaPipe worker thread.
- (void)mediapipeGraph:(MPPGraph*)graph
    didOutputPixelBuffer:(CVPixelBufferRef)pixelBuffer
              fromStream:(const std::string&)streamName {
  if (streamName == kOutputStream) {
    // Display the captured image on the screen.
    CVPixelBufferRetain(pixelBuffer);
    dispatch_async(dispatch_get_main_queue(), ^{
      [_renderer renderPixelBuffer:pixelBuffer];
      CVPixelBufferRelease(pixelBuffer);
    });
  }
}

// Receives a raw packet from the MediaPipe graph. Invoked on a MediaPipe worker thread.
- (void)mediapipeGraph:(MPPGraph*)graph
     didOutputPacket:(const ::mediapipe::Packet&)packet
          fromStream:(const std::string&)streamName {
  if (streamName == kLandmarksOutputStream) {
    if (packet.IsEmpty()) {
      NSLog(@"[TS:%lld] No hand landmarks", packet.Timestamp().Value());
      return;
    }
    const auto& multi_hand_landmarks = packet.Get<std::vector<::mediapipe::NormalizedLandmarkList>>();
    NSLog(@"[TS:%lld] Number of hand instances with landmarks: %lu", packet.Timestamp().Value(),
          multi_hand_landmarks.size());
      NSDate*currentDate=[NSDate date];
    for (int hand_index = 0; hand_index < multi_hand_landmarks.size(); ++hand_index) {
        //till h=1
      const auto& landmarks = multi_hand_landmarks[hand_index];
      NSLog(@"\tNumber of landmarks for hand[%d]: %d", hand_index, landmarks.landmark_size());
      
            
        
      for (int i = 0; i < landmarks.landmark_size(); ++i) {
          //till i=20
        NSLog(@"\t\tLandmark[%d]: (%f, %f, %f)", i, landmarks.landmark(i).x(),
              landmarks.landmark(i).y(), landmarks.landmark(i).z());
            NSDateFormatter *dateFormatter = [[NSDateFormatter alloc] init];
                    // フォーマットを指定の日付フォーマットに設定
                    [dateFormatter setDateFormat:@"HH:mm:ss.SSS"];
                    // 日付型の文字列を生成
        NSString *str_t = [dateFormatter stringFromDate:currentDate];

        NSString *str_h = [NSString stringWithFormat:@"%d", hand_index];
        NSString *str_i = [NSString stringWithFormat:@"%d", i];
        NSString *str_x = [NSString stringWithFormat:@"%f", landmarks.landmark(i).x()];
        NSString *str_y = [NSString stringWithFormat:@"%f", landmarks.landmark(i).y()];
        NSString *str_z = [NSString stringWithFormat:@"%f", landmarks.landmark(i).z()];

          [mstr_t appendString:@"\n"];
          [mstr_t appendString:str_t];
          
          [mstr_h appendString:@"\n"];
          [mstr_h appendString:str_h];

          [mstr_i appendString:@"\n"];
          [mstr_i appendString:str_i];
          
          [mstr_x appendString:@"\n"];
          [mstr_x appendString:str_x];
          
          [mstr_y appendString:@"\n"];
          [mstr_y appendString:str_y];
          
          [mstr_z appendString:@"\n"];
          [mstr_z appendString:str_z];
      
    }
  }
      NSData* out_data_t = [mstr_t dataUsingEncoding:NSUTF8StringEncoding];
      [out_data_t writeToFile: pth_t atomically:NO];
      
      NSData* out_data_h = [mstr_h dataUsingEncoding:NSUTF8StringEncoding];
      [out_data_h writeToFile: pth_h atomically:NO];
      
      NSData* out_data_i = [mstr_i dataUsingEncoding:NSUTF8StringEncoding];
      [out_data_i writeToFile: pth_i atomically:NO];
      
      NSData* out_data_x = [mstr_x dataUsingEncoding:NSUTF8StringEncoding];
      [out_data_x writeToFile: pth_x atomically:NO];
      
      NSData* out_data_y = [mstr_y dataUsingEncoding:NSUTF8StringEncoding];
      [out_data_y writeToFile: pth_y atomically:NO];
      
      NSData* out_data_z = [mstr_z dataUsingEncoding:NSUTF8StringEncoding];
      [out_data_z writeToFile: pth_z atomically:NO];
}
}
#pragma mark - MPPInputSourceDelegate methods

// Must be invoked on _videoQueue.
- (void)processVideoFrame:(CVPixelBufferRef)imageBuffer
                timestamp:(CMTime)timestamp
               fromSource:(MPPInputSource*)source {
  if (source != _cameraSource) {
    NSLog(@"Unknown source: %@", source);
    return;
  }
  [self.mediapipeGraph sendPixelBuffer:imageBuffer
                            intoStream:kInputStream
                            packetType:MPPPacketTypePixelBuffer];
}

@end

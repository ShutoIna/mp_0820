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
//
// A simple main function to run a MediaPipe graph.
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/commandlineflags.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"

#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/port/commandlineflags.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/map_util.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/ret_check.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/framework/port/statusor.h"

#include "mediapipe/framework/formats/landmark.pb.h"

constexpr char kWindowName[] = "MediaPipe";




DEFINE_string(
    calculator_graph_config_file, "",
    "Name of file containing text format CalculatorGraphConfig proto.");

DEFINE_string(input_side_packets, "",
              "Comma-separated list of key=value pairs specifying side packets "
              "for the CalculatorGraph. All values will be treated as the "
              "string type even if they represent doubles, floats, etc.");

// Local file output flags.
// Output stream
DEFINE_string(output_stream, "",
              "The output stream to output to the local file in csv format.");
DEFINE_string(output_stream_file, "",
              "The name of the local file to output all packets sent to "
              "the stream specified with --output_stream. ");
DEFINE_bool(strip_timestamps, false,
            "If true, only the packet contents (without timestamps) will be "
            "written into the local file.");
// Output side packets
DEFINE_string(output_side_packets, "",
              "A CSV of output side packets to output to local file.");
DEFINE_string(output_side_packets_file, "",
              "The name of the local file to output all side packets specified "
              "with --output_side_packets. ");



::mediapipe::Status OutputSidePacketsToLocalFile(
    ::mediapipe::CalculatorGraph& graph) {

    RET_CHECK(FLAGS_output_side_packets.empty() &&
              FLAGS_output_side_packets_file.empty())
        << "--output_side_packets and --output_side_packets_file should be "
           "specified in pair.";
      
        LOG(INFO) << "number22222";
  
  return ::mediapipe::OkStatus();
}

::mediapipe::Status RunMPPGraph() {
  std::string calculator_graph_config_contents;
  MP_RETURN_IF_ERROR(::mediapipe::file::GetContents(
      FLAGS_calculator_graph_config_file, &calculator_graph_config_contents));
  LOG(INFO) << "Get calculator graph config contents: "
            << calculator_graph_config_contents;
  ::mediapipe::CalculatorGraphConfig config =
      ::mediapipe::ParseTextProtoOrDie<::mediapipe::CalculatorGraphConfig>(
          calculator_graph_config_contents);
    
    
  std::map<std::string, ::mediapipe::Packet> input_side_packets;
  std::vector<std::string> kv_pairs =
      absl::StrSplit(FLAGS_input_side_packets, ',');
  for (const std::string& kv_pair : kv_pairs) {
      printf("%s\n",kv_pair.c_str());
       LOG(INFO) << "Init.";
    std::vector<std::string> name_and_value = absl::StrSplit(kv_pair, '=');
    RET_CHECK(name_and_value.size() == 2);
    RET_CHECK(!::mediapipe::ContainsKey(input_side_packets, name_and_value[0]));
    input_side_packets[name_and_value[0]] =
        ::mediapipe::MakePacket<std::string>(name_and_value[1]);
      
      printf("%s\n",name_and_value[0].c_str());
      printf("%s\n",name_and_value[1].c_str());
     
  
  }
    
    //initialize calc@60
  LOG(INFO) << "Initialize the calculator graph.";
     
  ::mediapipe::CalculatorGraph graph;
  MP_RETURN_IF_ERROR(graph.Initialize(config, input_side_packets));
  
    RET_CHECK(FLAGS_output_stream.empty() && FLAGS_output_stream_file.empty())
        << "--output_stream and --output_stream_file should be specified in "
           "pair.";
    
   
     cv::VideoCapture video("/Users/inagakishuto/Desktop/mv_hokan/0.mp4");
    RET_CHECK(video.isOpened());
    
    
    
    //a_o_r@94
    ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller multi_hand_landmarks_poller,
    graph.AddOutputStreamPoller("multi_hand_landmarks"));
    //startrun@98
    LOG(INFO) << "Start running the calculator graph.";
    MP_RETURN_IF_ERROR(graph.StartRun({}));
  
    
    //フレーム抽出
     int frame_num = video.get(cv::CAP_PROP_FRAME_COUNT);
    LOG(INFO)<< frame_num;

    cv::Mat frame;
    int ii;
    
    for (ii = 0; ii < frame_num; ii++) {

       // フレームを取得する
       video >> frame;
    
        cv::Mat camera_frame;
        
        cv::cvtColor(frame, camera_frame, cv::COLOR_BGR2RGB);
        
        // Wrap Mat into an ImageFrame.
        auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
            mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
            mediapipe::ImageFrame::kDefaultAlignmentBoundary);
        cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
        camera_frame.copyTo(input_frame_mat);

         mediapipe::Packet multi_hand_landmarks_packet;
        

         // Use packet.Get to recover values from packet
        
        
         const auto& multi_hand_landmarks = multi_hand_landmarks_packet.Get<mediapipe::NormalizedLandmarkList>();
         
    
        for (int i = 0; i < multi_hand_landmarks.landmark_size(); ++i)  { const mediapipe::NormalizedLandmark& landmark = multi_hand_landmarks.landmark(i); LOG(INFO) << "x coordinate: " << landmark.x(); LOG(INFO) << "y coordinate: " << landmark.y(); LOG(INFO) << "z coordinate: " << landmark.z(); }
           
    }
/*
    //挿入@100~153
        LOG(INFO) << "Start grabbing and processing frames.";
        size_t frame_timestamp = 0;
        bool grab_frames = true;
        while (grab_frames) {
          // Capture opencv camera or video frame.
          cv::Mat camera_frame_raw;
          capture >> camera_frame_raw;
          if (camera_frame_raw.empty()) break;  // End of video.
          cv::Mat camera_frame;
          cv::cvtColor(camera_frame_raw, camera_frame, cv::COLOR_BGR2RGB);
          if (!load_video) {
            cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*//* 1);
          }

          // Wrap Mat into an ImageFrame.
          auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
              mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
              mediapipe::ImageFrame::kDefaultAlignmentBoundary);
          cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
          camera_frame.copyTo(input_frame_mat);

          // Send image packet into the graph.
          MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
              kInputStream, mediapipe::Adopt(input_frame.release())
                                .At(mediapipe::Timestamp(frame_timestamp++))));

          // Get the graph result packet, or stop if that fails.
          mediapipe::Packet packet;
          mediapipe::Packet multi_hsnd_landmarks_packet;

          //Polling the poller to get landmark packet
          if (!poller.Next(&packet)) break;
          if (!poller_landmark.Next(&multi_hand_landmarks_packet)) break;

          // Use packet.Get to recover values from packet
          auto& output_frame = packet.Get<mediapipe::ImageFrame>();
          auto& output_landmarks = multi_hand_landmarks_packet.Get<::mediapipe::NormalizedLandmark>();

          // Convert back to opencv for display or saving.
          cv::Mat output_frame_mat = mediapipe::formats::MatView(&output_frame);
          cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2BGR);
          if (save_video) {
            writer.write(output_frame_mat);
          } else {
            cv::imshow(kWindowName, output_frame_mat);
            // Press any key to exit.
            const int pressed_key = cv::waitKey(5);
            if (pressed_key >= 0 && pressed_key != 255) grab_frames = false;
          }
          // printout landmark values
          for (const ::mediapipe::NormalizedLandmark& landmark : output_landmarks) {
                std::cout << landmark.DebugString();
          }
        }
        */
    
    
  MP_RETURN_IF_ERROR(graph.WaitUntilDone());
  return OutputSidePacketsToLocalFile(graph);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ::mediapipe::Status run_status = RunMPPGraph();
  if (!run_status.ok()) {
    LOG(ERROR) << "Failed to run the graph: " << run_status.message();
    return EXIT_FAILURE;
  } else {
    LOG(INFO) << "Success!";
  }
  return EXIT_SUCCESS;
}

/* Copyright 2022 iwatake2222

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
/*** Include ***/
/* for general */
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <array>
#include <algorithm>
#include <chrono>
#include <fstream>

/* for OpenCV */
#include <opencv2/opencv.hpp>

/* for My modules */
#include "common_helper.h"
#include "common_helper_cv.h"
#include "inference_helper.h"
#include "engine_depth_lapdepth.hpp"

/*** Macro ***/
#define TAG "DepthEngine"
#define PRINT(...)   COMMON_HELPER_PRINT(TAG, __VA_ARGS__)
#define PRINT_E(...) COMMON_HELPER_PRINT_E(TAG, __VA_ARGS__)

/* Model parameters */
#if defined(INFERENCE_HELPER_ENABLE_TFLITE) || defined(INFERENCE_HELPER_ENABLE_TFLITE_DELEGATE_XNNPACK) || defined(INFERENCE_HELPER_ENABLE_TFLITE_DELEGATE_GPU) || defined(INFERENCE_HELPER_ENABLE_TFLITE_DELEGATE_NNAPI) || defined(INFERENCE_HELPER_ENABLE_ARMNN)
#define MODEL_NAME  "lapdepth/ldrn_kitti_resnext101_pretrained_data_grad_192x320.tflite"
#define INPUT_NAME  "input_1"
#define INPUT_DIMS  { 1, 192, 320, 3 }
#define IS_NCHW     false
#define IS_RGB      true
#define OUTPUT_NAME0 "Identity"
#define OUTPUT_NAME1 "Identity_1"  // not in use
#define OUTPUT_NAME2 "Identity_2"  // not in use
#define OUTPUT_NAME3 "Identity_3"  // not in use
#define OUTPUT_NAME4 "Identity_4"  // not in use
#define OUTPUT_NAME5 "Identity_5"  // not in use
#define TENSORTYPE  TensorInfo::kTensorTypeFp32
#elif defined(INFERENCE_HELPER_ENABLE_TENSORRT) || defined(INFERENCE_HELPER_ENABLE_ONNX_RUNTIME) || defined(INFERENCE_HELPER_ENABLE_ONNX_RUNTIME_CUDA) || defined(INFERENCE_HELPER_ENABLE_OPENCV)
#define MODEL_NAME  "lapdepth/ldrn_kitti_resnext101_pretrained_data_grad_192x320.onnx"
#define INPUT_DIMS  { 1, 3, 192, 320 }
#define INPUT_NAME  "input.1"
#define IS_NCHW     true
#define IS_RGB      true
#define OUTPUT_NAME0 "2499"
#define OUTPUT_NAME1 "2489"  // not in use
#define OUTPUT_NAME2 "2491"  // not in use
#define OUTPUT_NAME3 "2493"  // not in use
#define OUTPUT_NAME4 "2495"  // not in use
#define OUTPUT_NAME5 "2497"  // not in use
#define TENSORTYPE  TensorInfo::kTensorTypeFp32
#else
// In case unsupported framework is selected, set dummy to avoid build error but will cause runtime error
#define MODEL_NAME  "dummy"
#define INPUT_DIMS  { 1, 1, 1, 1 }
#define INPUT_NAME  "dummy"
#define IS_NCHW     true
#define IS_RGB      true
#define OUTPUT_NAME0 "dummy"
#define OUTPUT_NAME1 "dummy"  // not in use
#define OUTPUT_NAME2 "dummy"  // not in use
#define OUTPUT_NAME3 "dummy"  // not in use
#define OUTPUT_NAME4 "dummy"  // not in use
#define OUTPUT_NAME5 "dummy"  // not in use
#define TENSORTYPE  TensorInfo::kTensorTypeFp32
#endif

/*** Function ***/
int32_t DepthEngine::Initialize(const std::string& work_dir, const int32_t num_threads)
{
    /* Set model information */
    std::string model_filename = work_dir + "/model/" + MODEL_NAME;

    /* Set input tensor info */
    input_tensor_info_list_.clear();
    InputTensorInfo input_tensor_info(INPUT_NAME, TENSORTYPE, IS_NCHW);
    input_tensor_info.tensor_dims = INPUT_DIMS;
    input_tensor_info.data_type = InputTensorInfo::kDataTypeImage;
    input_tensor_info.normalize.mean[0] = 0.485f;
    input_tensor_info.normalize.mean[1] = 0.456f;
    input_tensor_info.normalize.mean[2] = 0.406f;
    input_tensor_info.normalize.norm[0] = 0.229f;
    input_tensor_info.normalize.norm[1] = 0.224f;
    input_tensor_info.normalize.norm[2] = 0.225f;
    input_tensor_info_list_.push_back(input_tensor_info);

    /* Set output tensor info */
    output_tensor_info_list_.clear();
    output_tensor_info_list_.push_back(OutputTensorInfo(OUTPUT_NAME0, TENSORTYPE, IS_NCHW));
    output_tensor_info_list_.push_back(OutputTensorInfo(OUTPUT_NAME1, TENSORTYPE, IS_NCHW));
    output_tensor_info_list_.push_back(OutputTensorInfo(OUTPUT_NAME2, TENSORTYPE, IS_NCHW));
    output_tensor_info_list_.push_back(OutputTensorInfo(OUTPUT_NAME3, TENSORTYPE, IS_NCHW));
    output_tensor_info_list_.push_back(OutputTensorInfo(OUTPUT_NAME4, TENSORTYPE, IS_NCHW));
    output_tensor_info_list_.push_back(OutputTensorInfo(OUTPUT_NAME5, TENSORTYPE, IS_NCHW));

    /* Create and Initialize Inference Helper */
#if defined(INFERENCE_HELPER_ENABLE_OPENCV)
    inference_helper_.reset(InferenceHelper::Create(InferenceHelper::kOpencv));
#elif defined(INFERENCE_HELPER_ENABLE_TFLITE)
    inference_helper_.reset(InferenceHelper::Create(InferenceHelper::kTensorflowLite));
#elif defined(INFERENCE_HELPER_ENABLE_TFLITE_DELEGATE_XNNPACK)
    inference_helper_.reset(InferenceHelper::Create(InferenceHelper::kTensorflowLiteXnnpack));
#elif defined(INFERENCE_HELPER_ENABLE_TFLITE_DELEGATE_GPU)
    inference_helper_.reset(InferenceHelper::Create(InferenceHelper::kTensorflowLiteGpu));
#elif defined(INFERENCE_HELPER_ENABLE_TENSORRT)
    inference_helper_.reset(InferenceHelper::Create(InferenceHelper::kTensorrt));
    if (inference_helper_) {
        InferenceHelperTensorRt* p = dynamic_cast<InferenceHelperTensorRt*>(inference_helper_.get());
        if (p) {
            p->SetDlaCore(-1);  /* Use GPU */
        }
    }
#elif defined(INFERENCE_HELPER_ENABLE_ARMNN)
    inference_helper_.reset(InferenceHelper::Create(InferenceHelper::kArmnn));
#elif defined(INFERENCE_HELPER_ENABLE_ONNX_RUNTIME)
    inference_helper_.reset(InferenceHelper::Create(InferenceHelper::kOnnxRuntime));
#elif defined(INFERENCE_HELPER_ENABLE_ONNX_RUNTIME_CUDA)
    inference_helper_.reset(InferenceHelper::Create(InferenceHelper::kOnnxRuntimeCuda));
#else
    PRINT_E("Inference Helper type is not selected\n");
#endif

    if (!inference_helper_) {
        return kRetErr;
    }
    if (inference_helper_->SetNumThreads(num_threads) != InferenceHelper::kRetOk) {
        inference_helper_.reset();
        return kRetErr;
    }
    if (inference_helper_->Initialize(model_filename, input_tensor_info_list_, output_tensor_info_list_) != InferenceHelper::kRetOk) {
        inference_helper_.reset();
        return kRetErr;
    }

    return kRetOk;
}

int32_t DepthEngine::Finalize()
{
    if (!inference_helper_) {
        PRINT_E("Inference helper is not created\n");
        return kRetErr;
    }
    inference_helper_->Finalize();
    return kRetOk;
}


int32_t DepthEngine::Process(const cv::Mat& original_mat, Result& result)
{
    if (!inference_helper_) {
        PRINT_E("Inference helper is not created\n");
        return kRetErr;
    }
    /*** PreProcess ***/
    const auto& t_pre_process0 = std::chrono::steady_clock::now();
    InputTensorInfo& input_tensor_info = input_tensor_info_list_[0];
    /* do resize and color conversion here because some inference engine doesn't support these operations */
    int32_t crop_x = 0;
    int32_t crop_y = 0;
    int32_t crop_w = original_mat.cols;
    int32_t crop_h = original_mat.rows;
    cv::Mat img_src = cv::Mat::zeros(input_tensor_info.GetHeight(), input_tensor_info.GetWidth(), CV_8UC3);
    CommonHelper::CropResizeCvt(original_mat, img_src, crop_x, crop_y, crop_w, crop_h, IS_RGB, CommonHelper::kCropTypeCut);

    input_tensor_info.data = img_src.data;
    input_tensor_info.data_type = InputTensorInfo::kDataTypeImage;
    input_tensor_info.image_info.width = img_src.cols;
    input_tensor_info.image_info.height = img_src.rows;
    input_tensor_info.image_info.channel = img_src.channels();
    input_tensor_info.image_info.crop_x = 0;
    input_tensor_info.image_info.crop_y = 0;
    input_tensor_info.image_info.crop_width = img_src.cols;
    input_tensor_info.image_info.crop_height = img_src.rows;
    input_tensor_info.image_info.is_bgr = false;
    input_tensor_info.image_info.swap_color = false;
    if (inference_helper_->PreProcess(input_tensor_info_list_) != InferenceHelper::kRetOk) {
        return kRetErr;
    }
    const auto& t_pre_process1 = std::chrono::steady_clock::now();

    /*** Inference ***/
    const auto& t_inference0 = std::chrono::steady_clock::now();
    if (inference_helper_->Process(output_tensor_info_list_) != InferenceHelper::kRetOk) {
        return kRetErr;
    }
    const auto& t_inference1 = std::chrono::steady_clock::now();

    /*** PostProcess ***/
    const auto& t_post_process0 = std::chrono::steady_clock::now();
    /* Retrieve the result */
    int32_t output_height = output_tensor_info_list_[0].GetHeight();
    int32_t output_width = output_tensor_info_list_[0].GetWidth();
    // int32_t output_channel = 1;
    float* values = output_tensor_info_list_[0].GetDataAsFloat();
    //printf("%f, %f, %f\n", values[0], values[100], values[400]);
    cv::Mat mat_out = cv::Mat(output_height, output_width, CV_32FC1, values);  /* value has no specific range */

    const auto& t_post_process1 = std::chrono::steady_clock::now();

    /* Return the results */
    result.mat_out = mat_out;
    result.time_pre_process = static_cast<std::chrono::duration<double>>(t_pre_process1 - t_pre_process0).count() * 1000.0;
    result.time_inference = static_cast<std::chrono::duration<double>>(t_inference1 - t_inference0).count() * 1000.0;
    result.time_post_process = static_cast<std::chrono::duration<double>>(t_post_process1 - t_post_process0).count() * 1000.0;;

    return kRetOk;
}


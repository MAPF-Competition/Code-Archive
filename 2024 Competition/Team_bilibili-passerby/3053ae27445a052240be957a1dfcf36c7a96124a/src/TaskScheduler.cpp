#include "TaskScheduler.h"

#include "scheduler.h"
#include "const.h"
#include <vector>
#include <algorithm>
#include <onnxruntime_cxx_api.h>
// #include <nanoflann.hpp> // nanoflann for KD-Tree implementation


// Helper function to convert a 1D map into a 2D representation
std::vector<std::vector<int>> transfer_1d_map_to_2d_map(const std::vector<int> &map_1d, int rows, int cols)
{
    std::vector<std::vector<int>> map_2d(rows, std::vector<int>(cols));
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            map_2d[i][j] = map_1d[i * cols + j];
        }
    }
    return map_2d;
}

// Function to convert a 1D position to 2D coordinates
std::pair<int, int> one_d_to_two_d(int position, int cols)
{
    int x = position / cols;
    int y = position % cols;
    return {x, y};
}


/**
 * Initializes the task scheduler with a given time limit for preprocessing.
 *
 * This function prepares the task scheduler by allocating up to half of the given preprocessing time limit
 * and adjust for a specified tolerance to account for potential timing errors.
 * It ensures that initialization does not exceed the allocated time.
 *
 * @param preprocess_time_limit The total time limit allocated for preprocessing (in milliseconds).
 *
 */
void TaskScheduler::initialize(int preprocess_time_limit)
{
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);


    // 使用模型创建会话
    // try
    // {
    Ort::Env onnx_runtime_env(ORT_LOGGING_LEVEL_WARNING, "ONNXRuntime");
    Ort::SessionOptions session_options;
    // OrtCUDAProviderOptions cuda_options;
    // cuda_options.device_id = 0; // 指定GPU设备ID
    // session_options.AppendExecutionProvider_CUDA(cuda_options);

    session_options.SetIntraOpNumThreads(8);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        // Ort::Env onnx_runtime_env(ORT_LOGGING_LEVEL_WARNING, "ONNXRuntime");
        // // check CUDA
        // Ort::SessionOptions session_options;
        // Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0));
        // session_options.SetIntraOpNumThreads(8);
        // session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        // // 第一个参数是设备ID, 通常0是默认的GPU
        // OrtCUDAProviderOptions cuda_options;
        // cuda_options.device_id = 0; // 使用GPU设备ID 0
        // session_options.AppendExecutionProvider_CUDA(cuda_options);
        // // session_options.AppendExecutionProvider_CUDA(0);

        // // 可以设置其他CUDA相关的选项，如流和CUDA图等
        // session_options.AddConfigEntry("ORT_CUDA_STREAM", "0");
        // session_options.EnableCudaGraph();


        // const char *model_path = "simple_model.onnx";
        // std::cout << "loading model, model path: " << model_path << std::endl;
        // this->onnx_env = std::make_unique<Ort::Env>(std::move(onnx_runtime_env));
        // this->onnx_session = std::make_unique<Ort::Session>(*this->onnx_env, model_path, session_options);

        // // warm up
        // const char *input_names[] = {"input"};
        // const char *output_names[] = {"output"};
        // std::vector<float> input_tensor_values(env->num_of_agents * 351, 0.0); // Example input data (flattened)
        // std::vector<int64_t> input_shape = {env->num_of_agents, 351};
        // Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        // Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_tensor_values.data(), input_tensor_values.size(), input_shape.data(), input_shape.size());
        // std::vector<Ort::Value> output_tensors = this->onnx_session->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);
        // std::cout << "Warm up done." << std::endl;

        std::cout << "ONNX model loaded successfully." << std::endl;
    // }
    // catch (const Ort::Exception &e)
    // {
    //     std::cerr << "Error loading the model: " << e.what() << std::endl;
    // }

}

/**
 * Plans a task schedule within a specified time limit.
 *
 * This function schedules tasks by calling shedule_plan function in default planner with half of the given time limit,
 * adjusted for timing error tolerance. The planned schedule is output to the provided schedule vector.
 *
 * @param time_limit The total time limit allocated for scheduling (in milliseconds).
 * @param proposed_schedule A reference to a vector that will be populated with the proposed schedule (next task id for each agent).
 */

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
}

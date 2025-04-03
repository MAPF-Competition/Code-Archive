#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <string>
#include <vector>
#include "NewScheduler.h"

namespace DefaultPlanner
{
    // マルチプロセスを用いてタブーサーチでスケジュールを探索する
    // process_count: 生成する子プロセスの数 (例:4)
    SchedulingResult parallelTabuSearchMP(const std::vector<int> &initial_schedule,
                                          SharedEnvironment *env,
                                          const TabuParameters &params,
                                          const TimePoint &endtime,
                                          int process_count)
    {
        // 最良の結果を格納
        SchedulingResult best_result;
        best_result.cost = INT_MAX;
        best_result.schedule = initial_schedule;

        // プロセスごとに異なる乱数シードを用いるための基準時刻など
        unsigned long base_seed = (unsigned long)std::chrono::system_clock::now().time_since_epoch().count();

        // 子プロセスが出力に使う一時ファイル名を格納する配列
        std::vector<std::string> temp_files(process_count);

        // 子プロセス生成ループ
        for (int i = 0; i < process_count; i++)
        {
            std::string filename = "/tmp/tabu_result_" + std::to_string(i) + ".json";
            temp_files[i] = filename;

            pid_t pid = fork();
            if (pid < 0)
            {
                std::cerr << "[Error] fork() failed" << std::endl;
                continue;
            }
            else if (pid == 0)
            {
                // 子プロセスの実行パート
                // 子プロセスでは異なるシードを設定してperformTabuSearchを実行

                // シードを適当にずらす
                std::mt19937 local_mt(base_seed + i * 137);

                // タブーサーチ実行用のスケジュールをコピー
                std::vector<int> local_schedule = initial_schedule;
                // performTabuSearch でタブーサーチを実行
                SchedulingResult local_result = performTabuSearch(local_schedule, env, params, endtime);

                // 子プロセスの結果を一時ファイルに出力
                // JSONにまとめてもいいですが、ここでは簡易にテキストで書き込む
                std::ofstream ofs(filename);
                if (ofs)
                {
                    ofs << local_result.cost << std::endl;
                    for (auto &sc : local_result.schedule)
                    {
                        ofs << sc << " ";
                    }
                    ofs << std::endl;
                }
                ofs.close();

                // 子プロセス終了
                _exit(0);
            }
            // 親プロセスは何もしないで次へ(子の終了を待つのは後でまとめて行う)
        }

        // 親プロセス側: すべての子プロセス終了を待機
        for (int i = 0; i < process_count; i++)
        {
            int status;
            wait(&status); // 全子プロセスの終了を待つ
        }

        // 親プロセス側: 一時ファイルを読み込み、最良解を決定
        for (int i = 0; i < process_count; i++)
        {
            std::ifstream ifs(temp_files[i]);
            if (!ifs.good())
                continue;

            int cost;
            ifs >> cost;
            std::vector<int> sched(env->num_of_agents, -1);
            for (int a = 0; a < env->num_of_agents; a++)
            {
                ifs >> sched[a];
            }
            ifs.close();

            // より良いコストの結果があれば更新
            if (cost < best_result.cost)
            {
                best_result.cost = cost;
                best_result.schedule = sched;
            }

            // 一時ファイルは削除しておく
            std::remove(temp_files[i].c_str());
        }

        return best_result;
    }
}
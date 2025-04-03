#include "WeightTable.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <boost/filesystem.hpp>

// コンストラクタ: 行数と列数をパラメータとして重みベクターを初期化
WeightTable::WeightTable(int rows, int cols)
    : rows_(rows), cols_(cols)
{
    // 各セルごとに5つの重み（右, 下, 左, 上, 待機）を用意。初期値は 1.0f に設定。
    weights_ = std::make_shared<std::vector<float>>(rows_ * cols_ * 5, 1.0f);
}

WeightedCostType WeightTable::getWeight(int loc, int from_dir) const
{
    int weight_idx = loc * 5 + from_dir;
    return weights_->at(weight_idx);
}

std::string WeightTable::loadWeights(const std::string &weights_path)
{
    std::string suffix = "all_one";

    if (!weights_path.empty())
    {
        std::ifstream file(weights_path);
        if (!file)
        {
            std::cerr << "Failed to open file: " << weights_path << std::endl;
            exit(-1);
        }

        try
        {
            // JSONをパース
            nlohmann::json json_weights = nlohmann::json::parse(file);
            if (json_weights.size() != weights_->size())
            {
                std::cerr << "map weights size mismatch" << std::endl;
                exit(-1);
            }

            // JSON内の各値を重みベクターに格納
            for (std::size_t i = 0; i < weights_->size(); ++i)
            {
                (*weights_)[i] = json_weights[i].get<float>();
            }
        }
        catch (nlohmann::json::parse_error &error)
        {
            std::cerr << "Failed to load " << weights_path << std::endl;
            std::cerr << "Message: " << error.what() << std::endl;
            exit(1);
        }

        // boost::filesystem を利用してファイルパスから stem を取得
        boost::filesystem::path bf_path(weights_path);
        suffix = bf_path.stem().string();
    }

    return suffix;
}
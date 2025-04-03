#ifndef FAST_DISTANCE_TABLE_H
#define FAST_DISTANCE_TABLE_H

#include <vector>
#include <limits>
#include <fstream>
#include <algorithm> // std::min用
#include <iostream>

namespace DefaultPlanner
{
    class FastDistanceTable
    {
    private:
        std::vector<int> dist;        // 三角行列形式の距離テーブル
        std::vector<int> cellToIndex; // セル位置からインデックスへの変換テーブル（サイズ = 全セル数）
        std::vector<int> indexToCell; // インデックスからセル位置への変換テーブル（サイズ = 通行可能セル数）
        int totalCells;               // 全セル数
        int passableCount;            // 通行可能なセル数

        // 三角行列のインデックスを計算する補助関数
        size_t getTriangleIndex(int row, int col) const
        {
            if (row > col)
                std::swap(row, col);

            // long longを使用して計算
            long long row_long = static_cast<long long>(row);
            long long col_long = static_cast<long long>(col);
            long long pc = static_cast<long long>(passableCount);
            long long base = row_long * pc - row_long * (row_long + 1) / 2;
            long long result = base + col_long;
            // 結果が範囲内かチェック
            if (result < 0 || result >= static_cast<long long>(dist.size()))
            {
                std::cout << "インデックス計算エラー:" << std::endl;
                std::cout << "row=" << row << ", col=" << col << std::endl;
                std::cout << "計算結果=" << result << ", dist.size()=" << dist.size() << std::endl;
                throw std::out_of_range("三角行列インデックスが範囲外です");
            }

            return static_cast<size_t>(result);
        }

        void saveAsUShort(std::ofstream &file, int value) const
        {
            // unsigned shortの最大値でクランプする
            value = std::min(value, static_cast<int>(std::numeric_limits<unsigned short>::max()));
            // std::cout << "value: " << value << std::endl;
            unsigned short ushortVal = static_cast<unsigned short>(value);
            file.write(reinterpret_cast<const char *>(&ushortVal), sizeof(ushortVal));
        }

        static int loadAsInt(std::ifstream &file)
        {
            unsigned short ushortVal;
            file.read(reinterpret_cast<char *>(&ushortVal), sizeof(ushortVal));
            return static_cast<int>(ushortVal);
        }

    public:
        FastDistanceTable() : totalCells(0), passableCount(0) {}

        FastDistanceTable(const std::vector<std::vector<int>> &distances,
                          const std::vector<int> &toIndex,
                          const std::vector<int> &toCell,
                          int total,
                          int passable)
            : cellToIndex(toIndex),
              indexToCell(toCell),
              totalCells(total),
              passableCount(passable)
        {
            std::cout << "FastDistanceTable コンストラクタ開始" << std::endl;
            std::cout << "総セル数: " << total << std::endl;
            std::cout << "通行可能セル数: " << passable << std::endl;
            std::cout << "distances.size(): " << distances.size() << std::endl;
            std::cout << "cellToIndex.size(): " << toIndex.size() << std::endl;

            // celltoindex  298, 818
            int i = 298, j = 818;
            std::cout << "indexToCell[" << i << "]: " << indexToCell[i] << std::endl;
            std::cout << "indexToCell[" << j << "]: " << indexToCell[j] << std::endl;

            // cellToIndexの内容をサンプリング表示
            std::cout << "cellToIndex サンプル:" << std::endl;
            for (int i = 0; i < std::min(10, total); ++i)
            {
                std::cout << "cell " << i << " -> index " << toIndex[i] << std::endl;
            }

            // long longを使用して計算
            long long pc = static_cast<long long>(passableCount);
            long long size = (pc * (pc + 1)) / 2;

            // サイズが大きすぎないかチェック
            if (size > std::numeric_limits<size_t>::max())
            {
                throw std::runtime_error("三角行列のサイズが大きすぎます");
            }

            std::cout << "三角行列サイズ: " << size << std::endl;

            try
            {
                dist.resize(static_cast<size_t>(size));
            }
            catch (const std::bad_alloc &e)
            {
                throw std::runtime_error("メモリ確保に失敗しました: " + std::string(e.what()));
            }
            std::cout << "dist配列リサイズ完了" << std::endl;

            // 三角行列への格納を修正
            for (int i = 0; i < passableCount; ++i)
            {
                for (int j = i; j < passableCount; ++j) // すべての組み合わせを処理
                {
                    try
                    {
                        // 常に小さい方のインデックスを先に
                        // int min_idx = std::min(i, j);
                        // int max_idx = std::max(i, j);

                        size_t idx = getTriangleIndex(i, j);
                        dist[idx] = distances[i][j]; // 元の順序で値を取得

                        if ((i == 298 && j == 818) || (i == 818 && j == 298))
                        {
                            std::cout << "Setting distance [" << i << "][" << j << "]: "
                                      << distances[i][j] << " at idx=" << idx << std::endl;
                            std::cout << "dist[" << idx << "]: " << dist[idx] << std::endl;
                        }
                    }
                    catch (const std::exception &e)
                    {
                        std::cout << "エラー発生位置: i=" << i << ", j=" << j << std::endl;
                        throw;
                    }
                }
            }
            std::cout << "FastDistanceTable コンストラクタ完了" << std::endl;
        }

        int getDistance(int from_cell, int to_cell) const
        {
            // 範囲チェック
            if (from_cell >= totalCells || to_cell >= totalCells || from_cell < 0 || to_cell < 0)
            {
                std::cout << "範囲外のセル: from=" << from_cell << ", to=" << to_cell
                          << ", totalCells=" << totalCells << std::endl;
                return std::numeric_limits<int>::max();
            }

            int from_idx = cellToIndex[from_cell];
            int to_idx = cellToIndex[to_cell];

            // デバッグ出力を追加
            // std::cout << "getDistance: cell(" << from_cell << "," << to_cell << ") -> "
            //           << "idx(" << from_idx << "," << to_idx << ")" << std::endl;

            if (from_idx == -1 || to_idx == -1)
            {
                std::cout << "通行不可能なセル検出: from_idx=" << from_idx
                          << ", to_idx=" << to_idx << std::endl;
                return std::numeric_limits<int>::max();
            }

            try
            {
                size_t triangle_idx = getTriangleIndex(from_idx, to_idx);
                int distance = dist[triangle_idx];

                // 値の取得をデバッグ出力
                // std::cout << "Triangle index: " << triangle_idx
                //           << ", Distance: " << distance << std::endl;

                return distance;
            }
            catch (const std::exception &e)
            {
                std::cout << "インデックス計算エラー: " << e.what() << std::endl;
                return std::numeric_limits<int>::max();
            }
        }

        void save(std::ofstream &file) const
        {

            std::streampos start_pos = file.tellp();

            // サイズ情報はintのまま保存
            file.write(reinterpret_cast<const char *>(&totalCells), sizeof(totalCells));
            file.write(reinterpret_cast<const char *>(&passableCount), sizeof(passableCount));

            // セル変換テーブルもintのまま保存
            file.write(reinterpret_cast<const char *>(cellToIndex.data()), totalCells * sizeof(int));
            file.write(reinterpret_cast<const char *>(indexToCell.data()), passableCount * sizeof(int));

            // 距離データのみushortで保存
            for (int val : dist)
            {
                saveAsUShort(file, val);
            }

            std::streampos end_pos = file.tellp();
            std::cout << "Wrote " << (end_pos - start_pos) << " bytes" << std::endl;
            file.flush();

            // 重要な値のサンプルチェック（セル361とセル1023の距離）
            int from_cell = 361; // indexToCell[298]の値
            int to_cell = 1023;  // indexToCell[818]の値
            int distance = getDistance(from_cell, to_cell);
            std::cout << "Saved distance between cells " << from_cell << " and " << to_cell
                      << ": " << distance << std::endl;
        }

        static FastDistanceTable load(std::ifstream &file)
        {
            // サイズ情報の読み込み
            int total, passable;
            file.read(reinterpret_cast<char *>(&total), sizeof(total));
            file.read(reinterpret_cast<char *>(&passable), sizeof(passable));

            // 変換テーブルの読み込み
            std::vector<int> toIndex(total, -1);
            std::vector<int> toCell(passable);
            file.read(reinterpret_cast<char *>(toIndex.data()), total * sizeof(int));
            file.read(reinterpret_cast<char *>(toCell.data()), passable * sizeof(int));
            long long pc = static_cast<long long>(passable);

            // 三角行列サイズの計算
            long long size = (pc * (pc + 1)) / 2;

            // 距離データの読み込み
            std::vector<int> triangleData(static_cast<size_t>(size));

            // ファイルポジションの確認を追加
            std::streampos pos_before = file.tellg();
            std::cout << "Reading distance data from position: " << pos_before << std::endl;

            for (size_t i = 0; i < static_cast<size_t>(size); ++i)
            {
                triangleData[i] = loadAsInt(file);
                if (i % (size / 10) == 0)
                {
                    // 途中経過の値をチェック
                    std::cout << "Read value at " << i << ": " << triangleData[i] << std::endl;
                }
            }

            // 読み込み後のポジションも確認
            std::streampos pos_after = file.tellg();
            std::cout << "After reading, position: " << pos_after << std::endl;
            std::cout << "Bytes read: " << (pos_after - pos_before) << std::endl;

            // ファイル状態の詳細なチェック
            if (!file.good())
            {
                std::cout << "File status: " << std::endl;
                std::cout << "eof: " << file.eof() << std::endl;
                std::cout << "fail: " << file.fail() << std::endl;
                std::cout << "bad: " << file.bad() << std::endl;
                throw std::runtime_error("ファイルの読み込み中にエラーが発生しました");
            }

            // 読み込んだデータの簡単な検証
            size_t nonzero_count = 0;
            for (const auto &val : triangleData)
            {
                if (val != 0)
                    nonzero_count++;
            }
            std::cout << "Non-zero values in triangleData: " << nonzero_count
                      << " out of " << triangleData.size() << std::endl;

            // コンストラクタ用の空の distances を作成
            std::vector<std::vector<int>> distances(passable, std::vector<int>(passable));

            // FastDistanceTable の構築
            FastDistanceTable table(distances, toIndex, toCell, total, passable);
            table.dist = std::move(triangleData);

            // デバッグ用の確認（セル361とセル1023の距離）
            int from_cell = 361;
            int to_cell = 1023;
            int distance = table.getDistance(from_cell, to_cell);
            std::cout << "Loaded distance between cells " << from_cell << " and " << to_cell
                      << ": " << distance << std::endl;

            return table;
        }
    };
}

#endif
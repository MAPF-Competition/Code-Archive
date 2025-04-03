/*
Порядок выполнения тестов:
WAREHOUSE
SORTATION
RANDOM-05
RANDOM-04
RANDOM-03
RANDOM-02
RANDOM-01
GAME
CITY-02
CITY-01

он идет в обратном порядке от списка на сайте
*/


/*
ssh -i ../abc egor@51.250.101.48

scp -i ../abc -r * egor@51.250.101.48:/home/egor

./compile.sh

./build/lifelong -i ./example_problems/warehouse.domain/warehouse_large_5000.json -o test.json -s 1000 -t 300 -p 1800000

./build/lifelong -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 1000 -t 300 -p 1800000

./build/lifelong -i ./example_problems/my.domain/random_32_32_20_100.json -o test.json -s 1000 -t 300 -p 1800000

python3 PlanViz/script/run2.py --map example_problems/random.domain/maps/random-32-32-20.map --plan test.json --end 1000

-i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 1000 -t 500 -p 1800000
*/

/*
32 cores
use PIBTS
if PIBTS_STEPS enable, then PIBTS_STEPS=1000
if disable ENABLE_PIBTS_TRICK and PIBTS_STEPS=0, then PIBTS = PIBT2

1) DHM + PIBTS_STEPS + ENABLE_PIBTS_TRICK + ENABLE_SCHEDULER_TRICK
call(0): 2403, 11.3534s
call(1): 4184, 16.4483s
call(2): 5236, 23.3809s
call(3): 5797, 32.4601s
call(4): 5342, 87.6157s
call(5): 4193, 405.141s
total: 27155

=======================

2) DHM + ENABLE_PIBTS_TRICK + ENABLE_SCHEDULER_TRICK
call(0): 2350, 7.18267s
call(1): 3975, 11.0802s
call(2): 4827, 15.4018s
call(3): 5028, 21.9001s
call(4): 4634, 55.2668s
call(5): 3846, 150.983s
total: 24660

3) DHM + PIBTS_STEPS + ENABLE_SCHEDULER_TRICK
call(0): 2404, 12.0467s
call(1): 4192, 17.2747s
call(2): 5221, 23.7821s
call(3): 5624, 32.6402s
call(4): 4960, 86.9758s
call(5): 3764, 247.762s // TODO: попробовать увеличить PIBTS_STEPS так, чтобы сравнился с (1)
total: 26165

4) DHM + PIBTS_STEPS + ENABLE_PIBTS_TRICK
call(0): 2127, 12.2567s
call(1): 3704, 17.7565s
call(2): 4575, 24.9318s
call(3): 4951, 37.8259s
call(4): 4198, 169.524s
call(5): 3350, 495.446s
total: 22905

5) PIBTS_STEPS + ENABLE_PIBTS_TRICK + ENABLE_SCHEDULER_TRICK
call(0): 2372, 21.2071s
call(1): 3987, 37.6914s
call(2): 4190, 121.831s
call(3): 3471, 409.616s
call(4): 3092, 957.415s
call(5): 2604, 1967.65s
total: 19716

=======================

# тут все потоки выдают одинаковое решение, так как не используется рандом: disable ENABLE_PIBTS_TRICK + disable PIBTS_STEPS
6) DHM + ENABLE_SCHEDULER_TRICK
call(0): 2261, 7.07915s
call(1): 3701, 10.8508s
call(2): 4275, 14.6964s
call(3): 4176, 19.5032s
call(4): 3630, 34.707s
call(5): 3127, 52.4499s
total: 21170

7) DHM + PIBTS_STEPS
call(0): 2122, 12.1423s
call(1): 3699, 17.3189s
call(2): 4554, 23.6698s
call(3): 4959, 33.3806s
call(4): 4078, 111.868s
call(5): 3240, 234.339s
total: 22652

8) DHM + ENABLE_PIBTS_TRICK
call(0): 2052, 7.17383s
call(1): 3520, 11.0261s
call(2): 4192, 15.2919s
call(3): 4362, 21.9761s
call(4): 3773, 78.2991s
call(5): 3100, 233.486s
total: 20999

9) PIBTS_STEPS + ENABLE_PIBTS_TRICK
call(0): 2104, 21.1304s
call(1): 3387, 42.1391s
call(2): 3072, 208.643s
call(3): 2819, 510.713s
call(4): 2413, 1190.89s
call(5): 2206, 1807.67s
total: 16001

10) PIBTS_STEPS + ENABLE_SCHEDULER_TRICK

11) ENABLE_PIBTS_TRICK + ENABLE_SCHEDULER_TRICK

=======================

12) DHM
call(0): 1762, 6.95711s
call(1): 3227, 10.6797s
call(2): 3531, 14.3174s
call(3): 3355, 19.7687s
call(4): 2999, 33.3046s
call(5): 2511, 66.3698s
total: 17385

13) ENABLE_PIBTS_TRICK

14) ENABLE_SCHEDULER_TRICK
call(0): 2065, 4.87735s
call(1): 3009, 12.0905s
call(2): 2628, 66.2085s
call(3): 2585, 165.536s
call(4): 2352, 258.666s
call(5): 2161, 493.386s
total: 14800

15) PIBTS_STEPS

=======================

16)
call(0): 1355, 16.1625s
call(1): 2503, 14.6241s
call(2): 2254, 66.3312s
call(3): 2065, 172.373s
call(4): 1886, 406.353s
call(5): 1792, 586.784s
total: 11855


17) MAPFPLANNER + мой шедулер
call(0): 1840, 181.398s
call(1): 2877, 181.792s
call(2): 2743, 182.008s
call(3): 2650, 182.18s
call(4): 2395, 182.558s
call(5): 2019, 182.778s
total: 14524

18) MAPFPLANNER + их шедулер
call(0): 1756, 181.116s
call(1): 2773, 181.62s
call(2): 2636, 181.991s
call(3): 2557, 182.399s
call(4): 2229, 182.812s
call(5): 1838, 183.061s
total: 13789

19) MAPFPLANNER + мой шедулер + ENABLE_SCHEDULER_TRICK
call(0): 2107, 181.495s
call(1): 3392, 181.779s
call(2): 3737, 181.739s
call(3): 3239, 182.445s
call(4): 2868, 182.69s
call(5): 2330, 182.785s
total: 17673

20) MAPFPLANNER + мой шедулер + ENABLE_SCHEDULER_TRICK + DHM
call(0): 2096, 181.353s
call(1): 3361, 181.56s
call(2): 2944, 181.956s
call(3): 3117, 182.294s
call(4): 2935, 182.608s
call(5): 2440, 182.862s
total: 16893
*/

/*
32 cores
ENABLE_ALL

call(0): 2386, 80.9281s
call(1): 4277, 81.1942s
call(2): 5486, 81.2495s
call(3): 6313, 81.4385s
call(4): 6178, 81.8762s
call(5): 4388, 81.8032s
total: 29028
*/

/*
веса операций + без DHM
call(0): 1401, 48.7164s
call(1): 2536, 48.8561s
call(2): 3645, 65.4042s
call(3): 3885, 82.4048s
call(4): 5512, 165.276s
total: 16979

веса операций + DHM
power * power
call(0): 1394, 48.6156s
call(1): 2488, 48.7558s
call(2): 4742, 65.2078s
call(3): 4128, 81.9905s
call(4): 5235, 164.159s
total: 17987
power
call(0): 1416, 48.6326s
call(1): 2524, 48.7505s
call(2): 4985, 65.221s
call(3): 4086, 81.985s
call(4): 5010, 164.162s
total: 18021

call(0): 1421, 48.6268s
call(1): 2527, 48.7458s
call(2): 4901, 65.2095s
call(3): 4076, 81.9673s
call(4): 5178, 164.172s
total: 18103

DHM
call(0): 1435, 48.6835s
call(1): 2539, 48.7773s
call(2): 5086, 65.3003s
call(3): 3629, 82.0164s
call(4): 4688, 164.299s
total: 17377
*/

/*
call(0): 10173, 1525.63s
call(1): 46233, 3524.64s
call(2): 2621, 477.479s
call(3): 1611, 477.273s
call(4): 7159, 956.207s
call(5): 155049, 5209.88s ENTRY TIMEOUT
call(6): 5084, 956.684s
call(7): 6047, 1916.77s
call(8): 10174, 5087.78s
call(9): 102886, 5250.45s ENTRY TIMEOUT
total: 347037
*/

// RESULTS(32): (17994, 76, 13080) (17628, 77, 7410) (17537, 77, 8672) (17391, 77, 8853) (17357, 77, 8092) (17310, 77, 8057) (17282, 77, 6763) (17243, 77, 8825) (17235, 77, 10967) (17221, 76, 5428) (17203, 77, 5671) (17201, 77, 7013) (17191, 77, 6471) (17154, 76, 8447) (17151, 77, 6658) (17091, 77, 8786) (17088, 77, 6610) (17085, 76, 5958) (17056, 77, 4592) (17050, 77, 4277) (17037, 76, 8548) (16986, 76, 6697) (16980, 77, 5532) (16974, 77, 7466) (16935, 76, 8563) (16865, 76, 9151) (16836, 77, 11389) (16795, 76, 6616) (16690, 76, 3613) (16588, 76, 1543) (16503, 77, 6500) (15931, 76, 4561)
// without DHM:
// 4585(5184)

//RESULTS(32): (80455, 68, 32225) (80391, 68, 32175) (80320, 68, 31878) (80220, 67, 32595) (80220, 67, 34368) (80203, 67, 32043) (80105, 68, 34269) (80093, 67, 32866) (80062, 67, 31846) (79994, 68, 34212) (79978, 67, 30377) (79956, 67, 34806) (79917, 67, 31350) (79872, 68, 35397) (79827, 68, 35820) (79774, 68, 33441) (79626, 67, 34348) (79615, 67, 33771) (79607, 67, 32130) (79594, 67, 33524) (79570, 68, 30562) (79534, 67, 28237) (79519, 68, 36541) (79493, 68, 32319) (79473, 67, 35030) (79460, 68, 32490) (79425, 68, 29493) (79292, 68, 35179) (79277, 68, 32649) (79160, 68, 30488) (79124, 67, 36114) (78917, 68, 27977)
// DHM:
// 6322(6921)

//-i example_problems/warehouse.domain/warehouse_large_5000.json -o test.json -s 1000 -t 500 -p 1000000000
//459.301s
//без DHM
//tasks: 19359
//score: 96726.9
//
//DHM:
//tasks: 19447
//score: 97174.2

//-i example_problems/warehouse.domain/warehouse_large_10000.json -o test.json -s 1000 -t 500 -p 1000000000
//time:  466.095s
//tasks: 35056 -> 35775 -> 37433 (planner depth=4). 37676
// revealed: 37538+GG, 26253-GG
//
//tasks: 34067, SCHEDULER_LNS_TIME = 100
//tasks: 34422, SCHEDULER_LNS_TIME = 0
//tasks: 22155 без GG

//RESULTS(32): (2.08149e+06, 279, 160391) (2.08114e+06, 278, 152021) (2.08093e+06, 278, 162391) (2.08062e+06, 278, 163015) (2.08038e+06, 279, 156560) (2.07976e+06, 279, 137154) (2.07974e+06, 278, 154802) (2.0797e+06, 279, 156653) (2.07965e+06, 279, 167466) (2.07953e+06, 278, 131978) (2.07946e+06, 279, 167840) (2.07936e+06, 279, 156012) (2.07933e+06, 278, 143395) (2.07928e+06, 279, 165060) (2.07928e+06, 279, 151222) (2.07926e+06, 279, 149279) (2.07924e+06, 278, 153622) (2.07918e+06, 279, 158463) (2.07913e+06, 279, 154501) (2.07902e+06, 278, 148703) (2.07897e+06, 278, 151760) (2.07896e+06, 278, 156884) (2.07896e+06, 279, 158810) (2.07875e+06, 279, 130304) (2.07855e+06, 278, 161916) (2.07825e+06, 279, 154384) (2.07802e+06, 279, 147780) (2.07762e+06, 279, 132626) (2.07737e+06, 279, 140815) (2.07724e+06, 279, 138376) (2.07644e+06, 279, 164366) (2.07631e+06, 279, 156761)
//best: 2.08149e+06

//-i example_problems/game.domain/brc202d_6500.json -o test.json -s 1000 -t 500 -p 1000000000
//ENABLE_SMART_PLANNER: 6758
//MY: 6524 -> 8231


// (revealed tasks)
//6642 -> 7045 -> 7097
//6979
/*
call(0): 2513, 11.6532s
call(1): 4728, 16.9672s
call(2): 5987, 29.2818s
call(3): 6291, 53.7082s
call(4): 5528, 80.692s
call(5): 4276, 81.9039s
call(6): 3441, 82.2837s
call(7): 2374, 82.4319s
total: 35138

call(0): 2553, 11.2305s
call(1): 4702, 17.3299s
call(2): 6045, 28.9299s
call(3): 6221, 57.6373s
call(4): 4919, 81.3932s
call(5): 4157, 81.9923s
call(6): 3405, 82.3248s
call(7): 2300, 82.537s
total: 34302

call(0): 2504, 81.0183s
call(1): 4420, 81.2488s
call(2): 5895, 81.4853s
call(3): 5875, 81.6767s
call(4): 5154, 81.9609s
call(5): 5236, 82.1345s
call(6): 4901, 82.1636s
call(7): 3245, 82.3681s
total: 37230

depth=3
call(0): 2491, 81.0475s
call(1): 4566, 81.2671s
call(2): 5892, 81.5076s
call(3): 6460, 81.7227s
call(4): 5470, 82.0347s
call(5): 5003, 82.2074s
call(6): 4802, 82.1904s
call(7): 2716, 82.4431s
total: 37400 (грамотные веса)

call(0): 2498, 80.9358s
call(1): 4474, 81.1327s
call(2): 5893, 81.4448s
call(3): 6160, 81.5743s
call(4): 6104, 81.7854s
call(5): 4949, 81.9894s
call(6): 4540, 81.9457s
call(7): 2764, 82.1668s
total: 37382 (earthquake)

call(0): 2365, 81.0639s
call(1): 3168, 81.3493s
call(2): 3023, 81.6327s
call(3): 4202, 81.8728s
call(4): 3770, 82.2404s
call(5): 2950, 82.505s
call(6): 4112, 82.261s
call(7): 2694, 82.5471s
total: 26284 (без весов)

=========================
depth=4

call(0): 2474, 81.1227s
call(1): 4537, 81.3853s
call(2): 5874, 81.6258s
call(3): 6249, 81.9486s
call(4): 5416, 82.1911s
call(5): 4618, 82.423s
call(6): 3926, 82.3474s
call(7): 1932, 82.6905s
total: 35026 (без весов)

v5.0.5
call(0): 2534, 81.8058s
call(1): 4640, 82.0787s
call(2): 6095, 82.1907s
call(3): 6626, 82.4402s
call(4): 6240, 82.6466s
call(5): 5009, 82.7628s
call(6): 4794, 82.9618s
call(7): 2466, 83.1042s
total: 38404

========================

depth=5

call(0): 2439, 81.9562s
call(1): 4542, 82.1606s
call(2): 6293, 82.4873s
call(3): 6897, 82.6578s
call(4): 5613, 82.8679s
call(5): 4920, 82.9883s
call(6): 3923, 83.1203s
call(7): 2101, 83.29s
total: 36728

Adaptive depth

depth 3-5
call(0): 2432, 82.0431s
call(1): 4556, 82.2977s
call(2): 6326, 82.6553s
call(3): 6905, 82.8498s
call(4): 5859, 83.1808s
call(5): 4890, 83.4137s
call(6): 4555, 83.6506s
call(7): 2554, 83.7633s
total: 38077
=========================

S
call(0): 2215, 80.039s
call(1): 3651, 79.2728s
call(2): 3927, 78.5544s
call(3): 3709, 77.8526s
call(4): 3199, 77.1011s
call(5): 2510, 76.2795s
call(6): 2122, 75.4736s
call(7): 1583, 74.7917s
total: 22916

call(0): 2239, 80.2611s
call(1): 3646, 79.5524s
call(2): 3981, 78.8291s
call(3): 3711, 78.0698s
call(4): 3235, 77.3462s
call(5): 2610, 76.5308s
call(6): 1631, 75.5848s
call(7): 1340, 74.8567s
total: 22393

S + SCHEDULER_TRICK
call(0): 2204, 180.332s
call(1): 3792, 179.405s
call(2): 3361, 178.656s
call(3): 3196, 177.911s
call(4): 2910, 177.164s
call(5): 2341, 176.312s
call(6): 1205, 175.574s
call(7): 690, 174.581s
total: 19699

=======================
P old
call(0): 2020, 81.0257s
call(1): 3551, 81.1804s
call(2): 4526, 81.3897s
call(3): 4859, 81.5661s
call(4): 4082, 81.7625s
call(5): 3077, 81.9594s
call(6): 2573, 82.187s
call(7): 1692, 82.4826s
total: 26380

P + моя матрица для их шедулера
call(0): 2016, 81.7311s
call(1): 3602, 81.9547s
call(2): 4607, 82.194s
call(3): 4817, 82.4195s
call(4): 4370, 82.6833s
call(5): 3498, 82.7736s
call(6): 2806, 82.9989s
call(7): 1776, 83.0816s
total: 27492

P
call(0): 1994, 81.3657s
call(1): 3537, 81.5991s
call(2): 4474, 81.8529s
call(3): 4683, 82.0032s
call(4): 3972, 82.2895s
call(5): 3072, 82.432s
call(6): 2657, 82.6024s
call(7): 1673, 82.6616s
total: 26062
*/

//-i example_problems/game.domain/brc202d_6500.json -o test.json -s 1000 -t 500 -p 1000000000
// revealed tasks: 16362 -> 17191 (max assigned 5000) -> 17762 (4500) -> 19036 (3000) -> 19451(3500)

// -i example_problems/warehouse.domain/warehouse_large_10000.json -o test.json -s 1000 -t 1000 -p 1000000000
// 26306: S + scheduler trick
// 21796: S
// 38242 -> 38797

// -i example_problems/game.domain/brc202d_6500.json -o test.json -s 1000 -t 1000 -p 1000000000

// -i example_problems/random.domain/random_32_32_20_400.json -o test.json -s 1000 -t 130 -p 1000000000
// 7084

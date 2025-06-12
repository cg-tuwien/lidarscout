#include "pc2hm.h"
#include "pc2hm_dl.h"

#include <iostream>
#include <filesystem>
#include <tuple>

#include <cuda.h>

#include "glm/glm.hpp"
#include "glm/vec2.hpp"

using namespace pc2hm;

struct TestParams
{
	std::string model_path_hm;
	std::string model_path_rgb;
	std::vector<coord> point_cloud_flat;
	std::vector<float> pts_values;
	std::vector<RGB> pts_values_rgb;
	float bb_size;
	int res_interp;
	int res_dl;
};

// https://stackoverflow.com/questions/22387586/measuring-execution-time-of-a-function-in-c
template<typename F, typename... Args>
void measure_print_mean(F func, std::string name, int num_iterations = 1000, Args&&... args) {
    auto startTimePoint = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_iterations; ++i)
        func(std::forward<Args>(args)...);

    auto stopTimePoint = std::chrono::high_resolution_clock::now();
    const auto timeSpan = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli>>(stopTimePoint - startTimePoint);
    float duration_ms = float(timeSpan.count()) / float(num_iterations);

    std::cout << name << " computation took " << duration_ms << " ms (mean over "
        << num_iterations << ")" << std::endl;
}

template<typename F, typename... Args>
void measure_print(F func, std::string name, int num_iterations = 1000, Args&&... args) {
    std::vector<double> durations;
    durations.reserve(num_iterations);

    for (int i = 0; i < num_iterations; ++i) {
        auto startTimePoint = std::chrono::high_resolution_clock::now();
        func(std::forward<Args>(args)...);
        auto stopTimePoint = std::chrono::high_resolution_clock::now();

        // Calculate the duration of this iteration in milliseconds
        const auto timeSpan = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(stopTimePoint - startTimePoint);
        durations.push_back(timeSpan.count());
    }

    // Sort the durations to find the median
    std::sort(durations.begin(), durations.end());
    double median_duration;
    if (num_iterations % 2 == 0) {
        median_duration = (durations[num_iterations / 2 - 1] + durations[num_iterations / 2]) / 2.0;
    } else {
        median_duration = durations[num_iterations / 2];
    }

    std::cout << name << " computation took " << median_duration << " ms (median over "
              << num_iterations << ")" << std::endl;
}

std::vector<float> get_values()
{
    auto vals = { 
        1.3245484829f, 1.2877199650f, 1.3240575790f, 1.3433064222f, 1.3426190615f, 1.2465703487f, 1.3128615618f, 1.2632658482f, 1.2430348396f, 1.2563912868f, 1.2656228542f, 1.2965588570f, 1.3293607235f, 1.3271020651f, 1.2688637972f, 1.3450742960f, 1.3326015472f, 1.2827112675f, 1.2413651943f, 1.3081474304f, 1.1120237112f, 1.3018621206f, 1.3198344707f, 1.0698919296f, 1.2314461470f, 1.0852124691f, 1.3162007332f, 1.2982285023f, 1.3431100845f, 1.3105045557f, 1.2328209877f, 1.3286732435f, 1.3218969107f, 1.3615733385f, 1.2123935223f, 1.1232194901f, 1.3522435427f, 1.2924342155f, 1.1626995802f, 1.2615964413f, 1.2773097754f, 1.3313248158f, 1.1219427586f, 1.0960154533f, 1.2670960426f, 1.3104063272f, 1.2033582926f, 1.2224109173f, 1.3471366167f, 1.3645197153f, 0.9754145145f, 1.3918218613f, 1.2334102392f, 1.2987195253f, 1.0601691008f, 1.0212781429f, 1.2752474546f, 1.3691354990f, 0.9701111913f, 1.3905451298f, 1.1350046396f, 1.2402849197f, 1.2536413670f, 1.2120988369f, 1.3055942059f, 1.2382225990f, 0.8954721689f, 1.2590429783f, 0.9631383419f, 1.0380719900f, 1.1646637917f, 1.3654035330f, 0.9728610516f, 1.3017640114f, 1.1704580784f, 0.9954491854f, 0.9767894745f, 1.2959696054f, 1.0936584473f, 1.1147735119f, 0.9006772041f, 0.7361767292f, 0.6904112697f, 0.6481812000f, 0.6768583059f, 0.9516478777f, 0.7324448228f, 0.7085799575f, 0.6305035353f, 0.8063963652f, 0.7504171133f, 0.7532650828f, 0.8507869244f, 0.8659111857f, 0.7246862650f, 0.7123118639f, 0.9583261609f, 0.8213241100f, 0.5801221728f, 0.6493597627f, 1.2788810730f, 1.2295801640f, 1.2374367714f, 1.1719312668f, 1.2159290314f, 1.1843056679f, 1.2438204288f, 1.2429366112f, 1.1661369801f, 1.2022778988f, 1.2108221054f, 1.1846984625f, 1.2090543509f, 1.2504005432f, 1.2647390366f, 1.2506951094f, 1.2630695105f, 1.3029425144f, 1.3263163567f, 1.2938091755f, 1.2859523296f, 1.2956749201f, 1.3186559677f, 1.3533239365f, 1.4098923206f, 1.3452706337f, 1.3051030636f, 1.3443868160f, 1.4106780291f, 1.3716889620f, 1.3680552244f, 1.2975409031f, 1.3320124149f, 1.3234680891f, 1.3218969107f, 1.2811399698f, 1.2308568954f, 1.2896842957f, 1.3343693018f, 1.2496148348f, 1.3425207138f, 1.3328963518f, 1.2834970951f, 1.2585518360f, 1.3606895208f, 1.3135490417f, 1.2610070705f, 1.2739707232f, 1.3544040918f, 1.3501811028f, 1.2774081230f, 1.3340747356f, 1.3103080988f, 1.3072636127f, 1.2083668709f, 1.2937107086f, 1.3330926895f, 1.3741440773f, 1.3642251492f, 1.3153166771f, 1.3749299049f, 1.2982285023f, 1.3977144957f, 1.3217005730f, 1.2950856686f, 1.3439940214f, 1.2825149298f, 1.3287715912f, 1.3195397854f, 1.3935894966f, 1.3880897760f, 1.3783671856f, 1.3126652241f, 1.4323823452f, 1.4106780291f, 1.3110938072f, 1.3544040918f, 1.3900541067f, 1.3155132532f, 1.4120529890f, 1.3643231392f, 1.3412441015f, 1.3615733385f, 1.3837685585f, 1.3190486431f, 1.3989911079f, 1.4006605148f, 1.4040980339f, 1.2949874401f, 1.4093030691f, 1.3503775597f, 1.3090313673f, 1.3578413725f, 1.3284769058f, 1.3431100845f, 1.3965358734f, 1.3159061670f, 1.3744387627f, 1.3367264271f, 1.3812153339f, 1.4308109283f, 1.3738495111f, 1.3964377642f, 1.3221914768f, 1.4188294411f, 1.3751262426f, 1.3130580187f, 1.3277894258f, 1.2292854786f, 1.3594127893f, 1.4009553194f, 1.3628501892f, 1.3817062378f, 1.4725499153f, 1.3914289474f, 1.3806259632f, 1.3889737129f, 1.3644213676f, 1.3736531734f, 1.2829078436f, 1.4499616623f, 1.2860503197f, 1.3272000551f, 1.3067724705f, 1.3068709373f, 1.0489733219f, 1.1843056679f, 1.3003890514f, 1.3194416761f, 1.3130580187f, 1.1714402437f, 1.2431330681f, 1.3403601646f, 1.2916483879f, 1.3101117611f, 1.2122952938f, 1.2186788321f, 1.3550914526f, 1.3380032778f, 1.3652071953f, 1.3635375500f, 1.3131563663f, 1.3283786774f, 1.3505740166f, 1.3597073555f, 1.3088350296f, 1.3566629887f, 1.3137454987f, 1.2647390366f, 1.2428383827f, 1.2611054182f, 1.3138437271f, 1.2296782732f, 1.3611805439f, 1.2478469610f, 1.3766975403f, 1.3799383640f, 1.3568593264f, 1.3241555691f, 1.3100135326f, 1.3275928497f, 1.3404583931f, 1.3853399754f, 1.3696266413f, 1.3909380436f, 1.3572521210f, 1.3630464077f, 1.4065532684f, 1.3905451298f, 1.3559756279f, 1.2272231579f, 1.1679046154f, 1.3833758831f, 1.2398921251f, 1.3711978197f, 1.3767958879f, 1.3936878443f, 1.3492971659f, 1.3730639219f, 1.3857328892f, 1.3538148403f, 1.3679569960f, 1.3330926895f, 1.3797421455f, 1.3546004295f, 1.3515560627f, 1.3601003885f, 1.3699212074f, 1.3574485779f, 1.2862468958f, 1.2618910074f, 1.2639533281f, 1.3560736179f, 1.2972462177f, 1.2459810972f, 1.2981301546f, 1.1505217552f, 1.0176445246f, 1.3283786774f, 1.3566629887f, 1.3124687672f, 1.2992105484f 
        //2.9419569969f, 3.0075607300f, 3.0890746117f, 2.6544985771f, 2.6991839409f, 3.0356488228f, 2.5893857479f, 2.9505012035f, 2.7101829052f, 3.0626561642f, 3.2860825062f, 3.2883410454f, 3.2631995678f, 3.2594678402f, 3.2569143772f, 3.1417145729f, 3.3430435658f, 3.2643783092f, 3.2786183357f, 3.3590519428f, 2.8785138130f, 3.2974746227f, 2.8509173393f, 2.8342213631f, 3.4063887596f, 3.3082773685f, 2.9764285088f, 3.3240892887f, 3.5250256062f, 3.0216050148f, 3.5131421089f, 2.9889011383f, 3.2049612999f, 2.8790047169f, 2.8407034874f, 2.8019106388f, 2.4832215309f, 2.4666242599f, 2.7749028206f, 2.4010202885f, 2.6395704746f, 2.4533658028f, 2.4154570103f, 2.6945676804f, 2.7678320408f, 3.0807268620f, 3.1557583809f, 2.1829957962f, 2.2123603821f, 2.6181612015f, 2.7212808132f, 2.7311017513f, 2.2296450138f, 2.9156370163f, 2.7883577347f, 2.4542496204f, 2.4632852077f, 2.4923548698f, 2.8666305542f, 2.7210843563f, 2.8164455891f, 3.3421597481f, 2.9929277897f, 3.5090172291f, 3.2362902164f, 3.1369023323f, 3.3823273182f, 2.6653995514f, 3.2275497913f, 2.6998713017f, 2.7473063469f, 2.6363298893f, 2.8601489067f, 3.1722579002f, 3.5604791641f, 3.5783529282f, 3.5529170036f, 3.1574282646f, 3.4321193695f, 3.3053314686f, 3.1869893074f, 3.5074460506f, 3.3616054058f, 3.3401954174f, 3.5676481724f, 3.4139509201f, 3.2845110893f, 3.4819116592f, 3.2552444935f, 3.4993929863f, 3.5466315746f, 3.0730664730f, 3.0518531799f, 3.1046895981f, 3.1164746284f, 3.1591959000f, 3.1472144127f, 3.0441927910f, 3.0863246918f, 2.7753944397f, 2.8915758133f, 2.7111651897f, 2.7599751949f, 2.7220666409f, 2.6382942200f, 2.8492474556f, 2.7383689880f, 2.7820725441f, 2.5757346153f, 2.5799579620f, 3.1950423717f
    };
    std::vector<float> values(vals);
    return values;
}

std::vector<RGB> get_values_rgb(const std::vector<float>& vals)
{
    std::vector<RGB> values(vals.size());
    for (size_t i = 0; i < vals.size(); ++i)
		values[i] = RGB{ vals[i], vals[i] + 0.1f, vals[i] - 0.1f };
    return values;
}

std::vector<coord> get_points_flat()
{
    auto vals = {
        0.3143187852, -0.3784101582, 0.4656494574, -0.3960780068, 0.3221853482, -0.3783217699, 0.3548399322, -0.3476608342, 0.3737452177, -0.3300126274, 0.3916978732, -0.4637441974, 0.4325038270, -0.3701802210, 0.3396567782, -0.4423935010, 0.3486429269, -0.4580087758, 0.3984350294, -0.4568695482, 0.3555077553, -0.4421872615, 0.4240774712, -0.4112709817, 0.3645135458, -0.3705828790, 0.3553113367, -0.3726256319, 0.4600122450, -0.4350867309, 0.3306313459, -0.3520409678, 0.4017348611, -0.3474545947, 0.3272038422, -0.4237140969, 0.3810127040, -0.4665136990, 0.4480012506, -0.3494286011, 0.3241397128, -0.2208726599, 0.4446719562, -0.2047663388, 0.4635772417, -0.2124266622, 0.3896747621, -0.1779944904, 0.3741576966, -0.2081250960, 0.3530427025, -0.2126918273, 0.3363471257, -0.2808392433, 0.3484268665, -0.2628571250, 0.4148065156, -0.2794152088, 0.3255735682, -0.2878612065, 0.3296001485, -0.2523290907, 0.4518019496, -0.3084949752, 0.3502142753, -0.2891084643, 0.4347921031, -0.2406520079, 0.3756799404, 0.1500735934, 0.3181685888, 0.0613120505, 0.4161519827, -0.0777817460, 0.3793235045, 0.0210364267, 0.4498574059, -0.1042196828, 0.4078140152, -0.0027105760, 0.3382720275, 0.1279765065, 0.3954003628, -0.0314269681, 0.3455002301, -0.1464987758, 0.3713980160, 0.1201590482, 0.3809734203, 0.0417684047, 0.3276556048, 0.1399875009, 0.4009688287, 0.0488001888, 0.3784003373, 0.0698660784, 0.4012536356, -0.0499001327, 0.3278127397, -0.0920417327, 0.4457031536, -0.1537662622, 0.4347528194, -0.0228631193, 0.3426521611, 0.0524830367, 0.3779289328, 0.0039185501, 0.3755424474, 0.1191769554, 0.4585882105, 0.1141781033, 0.3658786547, 0.0421121372, 0.3605262492, -0.0744131678, 0.4296164743, 0.1530984391, 0.3597700378, -0.0631092802, 0.3572362385, 0.1088551606, 0.3891051483, 0.0372311362, 0.3283921744, 0.0357579971, 0.4280549468, 0.0618816643, 0.3151437432, 0.1832094029, 0.3253182241, 0.2724227084, 0.4345367590, 0.2913476358, 0.3991814199, 0.1852619767, 0.4277701399, 0.2678756190, 0.3590432892, 0.2926538191, 0.3502830218, 0.2517889397, 0.3589156171, 0.1895831848, 0.4159555641, 0.2608536558, 0.3626377486, 0.1981568546, 0.4286343816, 0.2130453807, 0.3776244840, 0.2394734966, 0.3712310602, 0.2968375343, 0.3769664819, 0.1793399574, 0.4407828689, 0.2315381872, 0.3670964497, 0.2517889397, 0.3249548497, 0.4602774100, 0.3156053268, 0.4089139591, 0.3235799199, 0.3417977404, 0.4236551713, 0.3646215760, 0.3820930061, 0.3769763028, 0.3753656707, 0.4521555030, 0.4430023985, 0.3867284838, 0.4594328103, 0.4028249840, 0.4259434474, 0.3577076430, 0.3528757467, 0.4142761855, 0.4303628648, 0.3961271115, 0.3742362640, 0.3885846391, 0.3908925571, 0.4261791497, 0.4064292644, 0.4273085564, 0.4182241984, 0.3264672726, 0.4032472839, 0.3749139080, 0.3903229433, 0.4531768794, 0.4438568192, 0.4613773539, 0.4500243617, 0.3377711602, 0.4563981437, 0.3708284022, -0.2399252593, -0.1769141883, -0.1902411870, -0.2404457684, -0.2073394218, -0.2450223207, -0.2093723538, -0.2959929345, -0.1941204533, -0.2600090561, -0.1775329068, -0.2627687367, -0.1936883325, -0.2310274990, -0.2644088316, -0.2349460490, -0.2459454879, -0.3130715275, -0.2449142905, -0.2806428248, -0.2807410341, -0.2741806545, -0.2562476408, -0.3006382332, -0.2788946997, -0.2772742466, -0.2563851338, -0.2279437277, -0.2482828686, -0.2062689407, -0.3092413657, -0.1824630124, -0.1691654765, -0.2039512018, -0.2514059236, -0.0713686803, -0.2667953170, 0.1033161575, -0.3062656247, -0.0244050049, -0.2789045206, -0.1004189839, -0.2420269378, -0.1079614562, -0.1710216318, -0.1023929903, -0.2009165352, 0.0494287282, -0.1741446868, -0.0206632315, -0.2134382178, 0.0536615480, -0.2001897865, -0.1125183666, -0.2154711498, 0.0508134790, -0.1606605533, 0.0141912403, -0.1630175759, 0.1300192594, -0.1826594309, 0.0702490945, -0.2351424676, -0.1086783839, -0.2953251114, 0.2190754302, -0.2651748639, 0.1975970617, -0.2564145966, 0.2228073826, -0.2742297591, 0.2965821901, -0.2084393657, 0.2173960516, -0.2367039951, 0.1791238970, -0.2310569617, 0.2107276418, -0.2041181576, 0.2381967760, 0.1136674151, -0.2718727365, 0.0351589205, -0.2272759046, -0.0674894139, -0.2434902560, -0.1080596655, -0.2739449522, 0.1102399114, -0.2279633696, 0.0704160503, -0.3109600280, -0.0683831183, -0.2999311264, -0.0238746748, -0.3139357691, 0.1382197339, -0.2735128314, 0.1529020205, -0.3109894908, -0.0717124127, -0.2638785015, 0.0432611857, -0.2307525130, 0.0393426356, -0.2755752262, -0.0290306617, -0.2313614105, -0.1539430389, -0.2965821901, -0.0326349421, -0.2552459062, 0.0728123566, -0.2558646247, 0.0433004694, -0.1940418859, 0.0989949494, -0.2151470592, -0.0261433091, -0.2134087550, 0.1385045408, -0.1783971484, -0.0780469110, -0.1864699508, 0.0312109077, -0.1616622879, -0.0390971125, -0.1811371872, -0.0592987604, -0.2199887764, -0.0012963624, -0.2005040562, -0.1157494517, -0.2017218512, -0.0335679303, -0.1675253816, -0.0449994899, -0.1708153923, 0.1081284120, -0.1729268918, 0.1440631858, -0.0602317485, -0.0837037652, 0.0202507525, -0.0678822510, -0.1524698997, 0.0401872354, -0.0954103108, -0.1208661550, -0.0689723740, -0.0799619919, -0.1400857101, -0.0962156269, -0.0610665273, -0.0358267436, -0.0473859753, 0.0245424979, 0.1292237643, 0.0783415388, -0.0538972502, 0.1191474927, 0.1041214736, 0.0068157237, 0.0336366768, -0.1391429011, -0.0655055866, 0.1373554923, -0.0306314729, -0.0715650988, 0.1118701854, 0.1166627980, -0.1489834705, 0.0709758432, -0.0822895517, -0.1398794707, -0.0066978726, -0.1149735985, -0.1558090151, 0.1059383451, -0.1171636653, 0.0684813276, 0.1073918424, 0.0364258202, 0.1040036224, 0.0975316312, 0.0402068773, 0.0185124484, 0.0281664201, 0.0580220398, 0.1223491150, 0.0635512220, -0.0740399726, -0.0851867253, -0.1238320751, 0.1420400747, 0.0670769350, 0.0284905107, 0.0071299934, 0.1504958933, -0.1077159330, -0.0784397481, 0.0147903168, -0.0073951584, -0.0220970869, 0.0075817560, -0.0811012195, -0.0316823122, 0.1566732567, -0.0385176777, -0.0106458854, -0.1242052703, -0.0277342993, -0.0398631448, 0.1400366055, -0.1174386513, -0.0971682569, -0.0018757972, 0.1434837511, 0.0734703588, 0.0691000460, -0.0297966941, -0.0455789246, 0.0356892506, -0.0165188001, 0.1478737057, -0.0192391970, 0.0739614051, -0.1448881437, 0.1375813736, -0.1121255295, 0.1372474621, -0.0225488496, 0.1273086834, -0.0499492374, 0.1266703231, 0.0037123106, 0.1342226164, 0.0339804092, 0.0208891128, 0.0854715322, 0.1105640020, -0.1284871947, -0.1069793635, 0.1746357331, -0.0314760727, 0.2012602676, 0.0970405848, 0.2201557322, -0.0711035152, 0.2257536609, 0.0394604868, 0.2415948170, -0.1082659050, 0.2884897459, 0.1565652265, 0.2338657470, -0.0717516965, 0.1990309171, -0.0239728841, 0.2833042961, -0.1405472937, 0.2797196576, 0.0169018163, 0.2606572373, 0.1155530332, 0.1669263050, 0.0618227387, 0.1741348658, -0.0361508342, 0.2619044951, -0.0035453548, 0.1799881386, 0.3051460389, -0.2492354986, 0.1595016838, -0.3056665481, 0.2736012198, -0.2266768281, 0.2002388912, -0.3003730682, 0.1686842511, -0.2887156272, 0.2560119386, -0.2512291469, 0.2649882663, -0.2804758690, 0.1995121425, -0.2562869245, 0.1630666805, -0.2270500233, 0.2350540792, -0.2339148516, 0.2688969955, -0.2318622778, 0.3001962915, -0.1777784299, 0.3052442482, -0.1589517119, 0.2337478959, -0.1904965311, 0.2443937813, -0.1892001687, 0.2324220707, -0.2071822869, 0.2868987556, -0.2204110763, 0.1587749352, -0.2073688846, 0.3005596658, -0.1122826643, 0.1860378300, -0.1004877304, 0.3017479980, -0.0610665273, 0.1722983524, 0.0903230704, 0.2508166679, 0.0475627520, 0.2218252899, -0.0762398603, 0.1994630379, 0.0951647877, 0.2565520896, -0.0788129433, 0.2312337384, 0.0218515637, 0.1574883937, 0.0585720117, 0.2645856083, -0.0327527933, 0.1719644409, 0.0958129689, 0.1859789044, 0.0258977859, 0.1888171525, -0.1442203206, 0.2579957659, 0.0151929749, 0.2618161067, 0.1299996176, 0.2674336773, 0.0625494874, 0.2861818279, -0.1281532832, 0.2488033778, -0.0346678741, 0.2609518651, 0.0575899190, 0.1633416665, 0.0483189634, 0.3101252492, 0.0124136524, 0.3005891286, -0.0229711495, 0.1717090967, 0.1354993370, 0.1623301110, 0.0276655528, 0.2497560077, 0.0333125861, 0.3071691500, -0.0821815215, 0.1829442378, -0.0762300394, 0.2451598137, 0.1080302027, 0.2485873174, -0.0376828989, 0.2596555027, 0.0410711189, 0.2145676244, -0.1405865774, 0.2035092600, 0.1421775677, 0.2058957454, 0.1547778177, 0.2403868429, 0.1404490844, 0.2644186525, -0.1309326057, 0.3104493398, 0.2571315243, 0.3137295296, 0.2000915773, 0.2646248920, 0.1587749352, 0.2749761496, 0.2440402279, 0.2474480898, 0.2977115968, 0.2613152394, 0.2631222901, 0.2442268255, 0.3140437993, 0.2910530080, 0.3024551048, 0.1701966739, 0.1706189738, 0.1875011482, 0.1604052092, 0.1653254939, 0.1724554872, 0.2144203105, 0.2857104234 
        //-0.4590670830, -0.4703206383, -0.4621122867, -0.4537669535, -0.4568226942, -0.4154332136, -0.3472796594, -0.4190474172, -0.3576164925, -0.4339467872, -0.3728846589, -0.3429489373, -0.3419373818, -0.4522074720, -0.3812721402, -0.3945488066, -0.3666572701, -0.4580871386, -0.3938533622, -0.3590600666, -0.4550103238, -0.2354711618, -0.4602788423, -0.2901478458, -0.5006673044, -0.2965227531, -0.4647465459, -0.1779916259, -0.4537880276, -0.2116047734, -0.3495767334, -0.2990516419, -0.3760036218, -0.2746900128, -0.4216184542, -0.3101892898, -0.4117031025, -0.3066909936, -0.3993853065, -0.2655438648, -0.4663271014, 0.0314741290, -0.4989392303, -0.1544202746, -0.4995187674, -0.0271223328, -0.4868953973, 0.1214814975, -0.3897123067, -0.1459274229, -0.4466860648, -0.1200168493, -0.4194372875, 0.0537704989, -0.3595869184, -0.0350251104, -0.3570369555, -0.1583400523, -0.3448245298, 0.0375434622, -0.3590389925, -0.1428295341, -0.3560780852, 0.0467739064, -0.4144954173, -0.0510835545, -0.3835270661, 0.0756032392, -0.4301429170, 0.1160549235, -0.4057918249, 0.1610059226, -0.3078079195, -0.4726493235, -0.2379052173, -0.3889325660, -0.3061114566, -0.3530434186, -0.2819605682, -0.4247058060, -0.2993677530, -0.3640967702, -0.2401179950, -0.3971830658, -0.2952899198, -0.4592040645, 0.1561062004, -0.4640089533, 0.1597309411, -0.4377717316, 0.1481718117, -0.3460362891, 0.1627550707, -0.3389237892, -0.1569702375, -0.4721856939, -0.0412208880, -0.4878015824, 0.1123458865, -0.4761265456, -0.1436935711, -0.3404305855, 0.0597871469, -0.4009447879, -0.0630009431, -0.4142846765, 0.1235994419, -0.3376593448, 0.0679428134, -0.3898492882, -0.0054476480, -0.3813669735, 0.0166906663, -0.4319447502, 0.0822837205, -0.4544202498, 0.2653647352, -0.4722173050, 0.2094552179, -0.4736608790, 0.1697200520, -0.4301956021, -0.3273119747, -0.2320150137, -0.2143865511, -0.2220575139, -0.3230550118, -0.1742825890, -0.2362719766, -0.1722805520, -0.2618769760, -0.2235853842, -0.3060166232, -0.1969477552, -0.2121948474, -0.2994415123, -0.3114221231, -0.2446383838, -0.2261353471, -0.2976186049, -0.2200027917, -0.2819289571, -0.2489901800, -0.3366056411, -0.2811913645, -0.3106002343, -0.3071019380, -0.2553966984, -0.3179024008, -0.1441782748, -0.2815706978, -0.1063187015, -0.2494432726, -0.0654876838, -0.2841311978, 0.0585859247, -0.1774753111, -0.0796599983, -0.2529310318, -0.1647254966, -0.3371851782, 0.0086825183, -0.2086122549, -0.0509781841, -0.2903585865, -0.0064697406, -0.2320466248, -0.1422605341, -0.3143198083, -0.1397211082, -0.1720171260, -0.0913561092, -0.2697165314, 0.0185030367, -0.1943345700, -0.0570791284, -0.2900951606, 0.0130237775, -0.2289908841, -0.0863826278, -0.3111060120, -0.0556566285, -0.3026026233, 0.0733061651, -0.3136454379, 0.0751606836, -0.1412595156, -0.1728179408, -0.0932422388, -0.2114256437, -0.0887113130, -0.1908468108, -0.1063608496, -0.1848828480, -0.0015910926, -0.1996873847, -0.1281514418, -0.1885286627, -0.1539566449, -0.3177443452, -0.0949387017, -0.2665238093, 0.0635067209, -0.3340346041, -0.0880474796, -0.2938779568, -0.0562888507, -0.3018018085, -0.0287239624, -0.2999367530, 0.0465737027, -0.2347967914, 0.0643075357, -0.2480945319, -0.1320922935, -0.3005057530, -0.0051315369, -0.3271012339, -0.0024024444, -0.3070387158, -0.1111025162, -0.1663587373

    };
    std::vector<coord> points(vals);
    return points;
}

std::vector<float> get_values_test()
{
    auto vals = { 0.1f, 0.3f, 0.5f, 0.888f };
    std::vector<float> values(vals);
    return values;
}

std::vector<glm::vec2> get_points_test()
{
    std::vector<glm::vec2> points;
    points.push_back(glm::vec2(-0.5f, -0.5f));
    points.push_back(glm::vec2(-0.5f,  0.4f));
    points.push_back(glm::vec2( 0.5f, -0.5f));
    points.push_back(glm::vec2( 0.3f,  0.5f));

    return points;
}

std::vector<coord> vec2_to_realT(const std::vector<glm::vec2>& point_cloud)
{
    // convert point cloud to flat vector
    std::vector<coord> point_cloud_flat;
    point_cloud_flat.reserve(point_cloud.size() * 2);
    for (const glm::vec2& point : point_cloud)
    {
        point_cloud_flat.push_back(double(point.x));
        point_cloud_flat.push_back(double(point.y));
    }
    return point_cloud_flat;
}

int test_triangulation_interpolation(
    TestParams& params,
	int measure_iterations)
{
    std::cout << "Testing interpolation HM" << std::endl;
    HeightmapGenerator hm_gen(params.bb_size, params.res_interp, measure_iterations, 3);
    auto [hm_nn, hm_nn_f] = hm_gen.pts2hm(params.point_cloud_flat, params.pts_values, InterpolationType::NEAREST);
    auto [hm_lin, hm_lin_f] = hm_gen.pts2hm(params.point_cloud_flat, params.pts_values, InterpolationType::LINEAR);
	return 0;
}

int test_learned(
    TestParams& params,
	int measure_iterations)
{
    std::cout << "Testing learned HM" << std::endl;
    HeightmapGeneratorDL hm_gen_dl(params.model_path_hm, params.model_path_rgb, 
        params.bb_size, params.res_interp, params.res_dl, measure_iterations, 3);
    //auto auto [hm_nn_lin, hm_nn_f2] = hm_gen_dl.pts2hm_vec(point_cloud_flat, pts_values));  // test only HM
    auto [hm_nn_lin, rgb_nn_lin, hm_nn_f2] = hm_gen_dl.pts2hm_vec(
        params.point_cloud_flat, params.pts_values, params.pts_values_rgb);
    return 0;
}

int test_cuda_copy(
    TestParams& params,
    int measure_iterations)
{
    HeightmapGeneratorDL hm_gen_dl(params.model_path_hm, params.model_path_rgb, 
        params.bb_size, params.res_interp, params.res_dl, measure_iterations, 3);

    {
        std::cout << "Testing CUDA HM data copy" << std::endl;
        CUdeviceptr target_buffer_hm;
        hm_gen_dl.verbose_level = 0;
        size_t target_size = hm_gen_dl.get_num_elem_dl() * sizeof(float);
        cuMemAlloc(&target_buffer_hm, target_size);
        hm_gen_dl.pts2hm_cu(params.point_cloud_flat, params.pts_values, target_buffer_hm);
    }

    {
        std::cout << "Testing CUDA HM + RGB data copy" << std::endl;
        CUdeviceptr target_buffer_hm;
        CUdeviceptr target_buffer_rgb;
        size_t target_size = hm_gen_dl.get_num_elem_dl() * sizeof(float);
        size_t target_size_rgb = hm_gen_dl.get_num_elem_dl() * sizeof(float) * 3;
        cuMemAlloc(&target_buffer_hm, target_size);
        cuMemAlloc(&target_buffer_rgb, target_size_rgb);
        hm_gen_dl.pts2hm_cu(params.point_cloud_flat, params.pts_values, params.pts_values_rgb, 
            target_buffer_hm, target_buffer_rgb);
    }

    {
        std::cout << "Testing batched CUDA HM + RGB data copy" << std::endl;
        int num_batches = 100;
		std::vector<CUdeviceptr> target_buffer_hm(num_batches);
		std::vector<CUdeviceptr> target_buffer_rgb(num_batches);
		for (int i = 0; i < num_batches; ++i)
		{
			size_t target_size = hm_gen_dl.get_num_elem_dl() * sizeof(float);
			cuMemAlloc(&target_buffer_hm[i], target_size);
			size_t target_size_rgb = hm_gen_dl.get_num_elem_dl() * sizeof(float) * 3;
			cuMemAlloc(&target_buffer_rgb[i], target_size_rgb);
		}

		HMs hm_nn(num_batches, std::vector<float>(hm_gen_dl.get_num_elem_interp()));
		HMs hm_lin(num_batches, std::vector<float>(hm_gen_dl.get_num_elem_interp()));
		IMGs rgb_nn(num_batches, std::vector<RGB>(hm_gen_dl.get_num_elem_interp()));
		IMGs rgb_lin(num_batches, std::vector<RGB>(hm_gen_dl.get_num_elem_interp()));

		hm_gen_dl.hm2hm_cu_batched(hm_nn, hm_lin, rgb_nn, rgb_lin, target_buffer_hm, target_buffer_rgb);
    }

    // cuCtxSynchronize(); // TODO: test with async
    return 0;
}

int test_timings_interpolation(
    TestParams& params,
	int measure_iterations)
{
    std::cout << "Timing interpolation" << std::endl;
    HeightmapGeneratorDL hm_gen_dl_timing(params.model_path_hm, params.model_path_rgb, 
        params.bb_size, params.res_interp, params.res_dl, measure_iterations, 0);
    auto bound_pts2hm = [hm_gen_dl_timing, params]()
        mutable {hm_gen_dl_timing.pts2hm(params.point_cloud_flat, params.pts_values, InterpolationType::NEAREST); };
    measure_print(bound_pts2hm, "pts2hm()", measure_iterations);
    return 0;
}

int test_timings_learned(
    TestParams& params,
	int measure_iterations)
{
    std::cout << "Timing learned" << std::endl;
    
    HeightmapGeneratorDL hm_gen_dl_timing(params.model_path_hm, params.model_path_rgb, 
        params.bb_size, params.res_interp, params.res_dl, measure_iterations, 0);
    HeightmapGenerator hm_gen_context(params.bb_size, params.res_interp, measure_iterations, 0);

    CUdeviceptr target_buffer_hm;
    size_t target_size = hm_gen_dl_timing.get_num_elem_dl() * sizeof(float);
    cuMemAlloc(&target_buffer_hm, hm_gen_dl_timing.get_num_elem_dl() * sizeof(float));
    CUdeviceptr target_buffer_rgb;
    size_t target_size_rgb = hm_gen_dl_timing.get_num_elem_dl() * sizeof(float) * 3;
    cuMemAlloc(&target_buffer_rgb, target_size_rgb);

    // init buffer with zeros
    cuMemsetD32(target_buffer_hm, 0, hm_gen_dl_timing.get_num_elem_dl());
    cuMemsetD32(target_buffer_rgb, 0, hm_gen_dl_timing.get_num_elem_dl() * 3);

    // HM
    auto [hm_nn_context, hm_nn_context_f] = hm_gen_context.pts2hm(
        params.point_cloud_flat, params.pts_values, InterpolationType::NEAREST);
    auto [hm_lin_context, hm_lin_context_f] = hm_gen_context.pts2hm(
        params.point_cloud_flat, params.pts_values, InterpolationType::LINEAR);
    auto bound_hm2hm_cu = [hm_gen_dl_timing, hm_nn_context, hm_lin_context, target_buffer_hm]()
        mutable {hm_gen_dl_timing.hm2hm_cu(hm_nn_context, hm_lin_context, target_buffer_hm); };
    measure_print(bound_hm2hm_cu, "hm2hm_cu() HM warm-up", measure_iterations);
    measure_print(bound_hm2hm_cu, "hm2hm_cu() HM proper", measure_iterations);
    measure_print(bound_hm2hm_cu, "hm2hm_cu() HM again", measure_iterations);

    // HM + RGB
    auto interp_types = { InterpolationType::NEAREST, InterpolationType::LINEAR };
    auto [hms, imgs, Mask] = hm_gen_context.pts2hm(
        params.point_cloud_flat, params.pts_values, interp_types, params.pts_values_rgb, interp_types);
    auto bound_hm2hm_cu_rgb = [hm_gen_dl_timing, hms, imgs, target_buffer_hm, target_buffer_rgb]()
		mutable {hm_gen_dl_timing.hm2hm_cu(hms[0], hms[1], imgs[0], imgs[1], 
            target_buffer_hm, target_buffer_rgb); };
    measure_print(bound_hm2hm_cu_rgb, "hm2hm_cu() HM+RGB warm-up", measure_iterations);
    measure_print(bound_hm2hm_cu_rgb, "hm2hm_cu() HM+RGB proper", measure_iterations);
    measure_print(bound_hm2hm_cu_rgb, "hm2hm_cu() HM+RGB again", measure_iterations);

    // CUDA HM (+ RGB)
    auto bound_pts2hm_cu = [hm_gen_dl_timing, params, target_buffer_hm]()
        mutable {hm_gen_dl_timing.pts2hm_cu(params.point_cloud_flat, params.pts_values, target_buffer_hm); };
    measure_print(bound_pts2hm_cu, "pts2hm_cu() HM", measure_iterations);
    auto bound_pts2hm_cu_rgb = [hm_gen_dl_timing, params, target_buffer_hm, target_buffer_rgb]()
		mutable {hm_gen_dl_timing.pts2hm_cu(params.point_cloud_flat, params.pts_values, params.pts_values_rgb, target_buffer_hm, target_buffer_rgb); };
    measure_print(bound_pts2hm_cu_rgb, "pts2hm_cu() HM+RGB", measure_iterations);
    
    if (false)
    {
        // read CUDA buffer back to CPU
        std::vector<float> hm_dl(hm_gen_dl_timing.get_num_elem_dl());
        cuMemcpyDtoH(hm_dl.data(), target_buffer_hm, target_size);
        std::vector<float> hm_dl_rgb(hm_gen_dl_timing.get_num_elem_dl() * 3);
        cuMemcpyDtoH(hm_dl_rgb.data(), target_buffer_rgb, target_size_rgb);

        // print buffer
        std::cout << "hm_dl: ";
        for (float f : hm_dl)
		    std::cout << f << ", ";
        std::cout << std::endl;
        std::cout << "hm_dl_rgb: ";
        for (float f : hm_dl_rgb)
            std::cout << f << ", ";
        std::cout << std::endl;
    }

    return 0;
}


int test_timings_learned_batched(
    TestParams& params,
	int measure_iterations,
    int num_batches = 10)
{
    std::cout << "Timing learned batched, num_batches = " << num_batches << std::endl;
    HeightmapGeneratorDL hm_gen_dl_timing(params.model_path_hm, params.model_path_rgb, 
        params.bb_size, params.res_interp, params.res_dl, measure_iterations, 0);
    HeightmapGenerator hm_gen_context(params.bb_size, params.res_interp, measure_iterations, 0);

	std::vector<CUdeviceptr> target_buffer_hm(num_batches);
	std::vector<CUdeviceptr> target_buffer_rgb(num_batches);
	for (int i = 0; i < num_batches; ++i)
	{
		size_t target_size = hm_gen_dl_timing.get_num_elem_dl() * sizeof(float);
		cuMemAlloc(&target_buffer_hm[i], target_size);
		size_t target_size_rgb = hm_gen_dl_timing.get_num_elem_dl() * sizeof(float) * 3;
		cuMemAlloc(&target_buffer_rgb[i], target_size_rgb);
	}

	HMs hm_nn(num_batches, std::vector<float>(hm_gen_dl_timing.get_num_elem_interp()));
	HMs hm_lin(num_batches, std::vector<float>(hm_gen_dl_timing.get_num_elem_interp()));
	IMGs rgb_nn(num_batches, std::vector<RGB>(hm_gen_dl_timing.get_num_elem_interp()));
	IMGs rgb_lin(num_batches, std::vector<RGB>(hm_gen_dl_timing.get_num_elem_interp()));

	hm_gen_dl_timing.hm2hm_cu_batched(hm_nn, hm_lin, rgb_nn, rgb_lin, target_buffer_hm, target_buffer_rgb);

    auto bound_hm2hm_cu_batched = [hm_gen_dl_timing, hm_nn, hm_lin, rgb_nn, rgb_lin, target_buffer_hm, target_buffer_rgb]()
        mutable {hm_gen_dl_timing.hm2hm_cu_batched(hm_nn, hm_lin, rgb_nn, rgb_lin, target_buffer_hm, target_buffer_rgb); };
    measure_print(bound_hm2hm_cu_batched, "hm2hm_cu_batched() warm-up", measure_iterations);
    measure_print(bound_hm2hm_cu_batched, "hm2hm_cu_batched() proper", measure_iterations);
    measure_print(bound_hm2hm_cu_batched, "hm2hm_cu_batched() again", measure_iterations);
    return 0;
}

int main()
{
    std::string model_path_hm("./ipes_cnn.pt");
    std::string model_path_rgb("./ipes_cnn_rgb.pt");
    float bb_size = 1.0f;
    int res_interp = 96;
    int res_dl = 64;
    int measure_iterations = 1000;
    
	std::string pwd = std::filesystem::absolute(".").string();
	std::cout << "Current path: " << pwd << std::endl;

    // proper patch data
    std::vector<coord> point_cloud_flat = get_points_flat();
    std::vector<float> pts_values = get_values();

    // toy example with 2 triangles
    //std::vector<glm::vec2> point_cloud = get_points_test();
    //std::vector<coord> point_cloud_flat(vec2_to_realT(point_cloud));
    //std::vector<float> pts_values = get_values_test();

    std::vector<RGB> pts_values_rgb = get_values_rgb(pts_values);

    TestParams test_params(model_path_hm, model_path_rgb, 
        point_cloud_flat, pts_values, pts_values_rgb,
        bb_size, res_interp, res_dl);

    int res_triangulation_interpolation = test_triangulation_interpolation(test_params, measure_iterations);
	int res_learned = test_learned(test_params, measure_iterations);
	int res_cuda_copy = test_cuda_copy(test_params, measure_iterations);
	int res_timings_interpolation = test_timings_interpolation(test_params, measure_iterations);
	int res_timings_learned = test_timings_learned(test_params, measure_iterations);

    for (int i : {1, 2, 3, 4, 5, 10, /*20, 50, 100, 1000*/})
	{
		int res_timings_learned_batched = test_timings_learned_batched(test_params, measure_iterations, i);
	}

	return 0;
}
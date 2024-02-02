/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:31:45 GMT-04:00
 */

#ifdef MATLAB_MEX_FILE
#include <stdexcept>
#include <cmath>
#include<math.h>
/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
inline double Power(double x, double y) { return pow(x, y); }
inline double Sqrt(double x) { return sqrt(x); }

inline double Abs(double x) { return fabs(x); }

inline double Exp(double x) { return exp(x); }
inline double Log(double x) { return log(x); }

inline double Sin(double x) { return sin(x); }
inline double Cos(double x) { return cos(x); }
inline double Tan(double x) { return tan(x); }

inline double ArcSin(double x) { return asin(x); }
inline double ArcCos(double x) { return acos(x); }
inline double ArcTan(double x) { return atan(x); }

/* update ArcTan function to use atan2 instead. */
inline double ArcTan(double x, double y) { return atan2(y,x); }

inline double Sinh(double x) { return sinh(x); }
inline double Cosh(double x) { return cosh(x); }
inline double Tanh(double x) { return tanh(x); }

const double E	= 2.71828182845904523536029;
const double Pi = 3.14159265358979323846264;
const double Degree = 0.01745329251994329576924;

inline double Sec(double x) { return 1/cos(x); }
inline double Csc(double x) { return 1/sin(x); }

#endif

/*
 * Sub functions
 */
static void output1(double *p_output1,const double *var1,const double *var2)
{
  double t1541;
  double t1421;
  double t1544;
  double t1473;
  double t1562;
  double t1147;
  double t1176;
  double t1481;
  double t1569;
  double t1604;
  double t1637;
  double t1690;
  double t1769;
  double t1824;
  double t1852;
  double t1853;
  double t1900;
  double t1911;
  double t1916;
  double t1962;
  double t1976;
  double t1987;
  double t2066;
  double t881;
  double t995;
  double t2070;
  double t2119;
  double t2021;
  double t2083;
  double t2095;
  double t859;
  double t2142;
  double t2178;
  double t2199;
  double t2292;
  double t2109;
  double t2218;
  double t2238;
  double t808;
  double t2303;
  double t2310;
  double t2330;
  double t2468;
  double t2268;
  double t2369;
  double t2385;
  double t19;
  double t2473;
  double t2486;
  double t2489;
  double t2652;
  double t2670;
  double t2740;
  double t2773;
  double t2820;
  double t2842;
  double t2615;
  double t2749;
  double t2755;
  double t2760;
  double t2849;
  double t2856;
  double t2880;
  double t2881;
  double t2897;
  double t2875;
  double t2903;
  double t2930;
  double t2952;
  double t2957;
  double t2972;
  double t2947;
  double t2974;
  double t2975;
  double t2990;
  double t2998;
  double t3003;
  double t2986;
  double t3017;
  double t3057;
  double t3112;
  double t3131;
  double t3156;
  double t3311;
  double t3316;
  double t3282;
  double t3284;
  double t3294;
  double t3317;
  double t3319;
  double t3322;
  double t3346;
  double t3359;
  double t3320;
  double t3365;
  double t3374;
  double t3399;
  double t3407;
  double t3423;
  double t3381;
  double t3428;
  double t3452;
  double t3487;
  double t3489;
  double t3496;
  double t3476;
  double t3501;
  double t3511;
  double t3520;
  double t3526;
  double t3529;
  double t3666;
  double t3667;
  double t3669;
  double t3598;
  double t3614;
  double t3636;
  double t3640;
  double t3641;
  double t3648;
  double t3662;
  double t3675;
  double t3678;
  double t3710;
  double t3717;
  double t3725;
  double t3707;
  double t3743;
  double t3785;
  double t3791;
  double t3793;
  double t3795;
  double t3787;
  double t3804;
  double t3811;
  double t3821;
  double t3823;
  double t3825;
  double t3813;
  double t3831;
  double t3832;
  double t3848;
  double t3868;
  double t3887;
  double t4000;
  double t4012;
  double t4044;
  double t3949;
  double t3962;
  double t3993;
  double t4076;
  double t4077;
  double t4099;
  double t3931;
  double t3998;
  double t4051;
  double t4058;
  double t4059;
  double t4060;
  double t4065;
  double t4103;
  double t4106;
  double t4110;
  double t4115;
  double t4118;
  double t4109;
  double t4119;
  double t4125;
  double t4137;
  double t4147;
  double t4152;
  double t4134;
  double t4168;
  double t4184;
  double t4188;
  double t4201;
  double t4207;
  double t4187;
  double t4209;
  double t4216;
  double t4260;
  double t4264;
  double t4265;
  double t4382;
  double t4383;
  double t4390;
  double t4405;
  double t4424;
  double t4429;
  double t4440;
  double t4445;
  double t4472;
  double t4437;
  double t4481;
  double t4483;
  double t4501;
  double t4502;
  double t4528;
  double t4487;
  double t4531;
  double t4535;
  double t4537;
  double t4538;
  double t4542;
  double t4536;
  double t4546;
  double t4550;
  double t4564;
  double t4593;
  double t4595;
  double t4692;
  double t4703;
  double t4719;
  double t4734;
  double t4735;
  double t4748;
  double t4788;
  double t4745;
  double t4790;
  double t4827;
  double t4830;
  double t4853;
  double t4869;
  double t4829;
  double t4891;
  double t4892;
  double t4896;
  double t4921;
  double t4922;
  double t4988;
  double t4990;
  double t4991;
  double t5001;
  double t5002;
  double t5049;
  double t5052;
  double t5012;
  double t5065;
  double t5080;
  double t5097;
  double t5098;
  double t5101;
  double t5248;
  double t5256;
  double t5259;
  double t5261;
  double t5280;
  double t5316;
  double t5317;
  double t5314;
  double t5332;
  double t5373;
  double t5374;
  double t5402;
  double t5508;
  double t5509;
  double t5513;
  double t5532;
  double t5535;
  double t5546;
  double t5553;
  double t5554;
  double t5555;
  double t5559;
  double t5564;
  double t5566;
  double t5579;
  double t5619;
  double t5650;
  double t5673;
  double t5674;
  double t5682;
  double t5668;
  double t5690;
  double t5692;
  double t5737;
  double t5760;
  double t5782;
  double t5709;
  double t5786;
  double t5787;
  double t5790;
  double t5794;
  double t5804;
  double t5949;
  double t5954;
  double t5926;
  double t5929;
  double t5947;
  double t5948;
  double t5957;
  double t5973;
  double t5980;
  double t6011;
  double t6015;
  double t5978;
  double t6017;
  double t6021;
  double t6031;
  double t6033;
  double t6039;
  double t6027;
  double t6042;
  double t6044;
  double t6062;
  double t6065;
  double t6084;
  double t6059;
  double t6088;
  double t6102;
  double t6167;
  double t6172;
  double t6178;
  double t6321;
  double t6327;
  double t6300;
  double t6309;
  double t6316;
  double t6329;
  double t6340;
  double t6354;
  double t6359;
  double t6360;
  double t6341;
  double t6362;
  double t6364;
  double t6394;
  double t6398;
  double t6406;
  double t6389;
  double t6410;
  double t6411;
  double t6429;
  double t6430;
  double t6457;
  double t6428;
  double t6467;
  double t6470;
  double t6483;
  double t6487;
  double t6495;
  double t6660;
  double t6672;
  double t6684;
  double t6574;
  double t6593;
  double t6612;
  double t6616;
  double t6632;
  double t6646;
  double t6647;
  double t6687;
  double t6691;
  double t6697;
  double t6698;
  double t6700;
  double t6693;
  double t6709;
  double t6730;
  double t6744;
  double t6748;
  double t6756;
  double t6731;
  double t6757;
  double t6758;
  double t6761;
  double t6770;
  double t6784;
  double t6760;
  double t6789;
  double t6790;
  double t6839;
  double t6843;
  double t6845;
  double t6900;
  double t6901;
  double t6905;
  double t6906;
  double t6913;
  double t6914;
  double t6932;
  double t6933;
  double t6947;
  double t6915;
  double t6980;
  double t6988;
  double t6999;
  double t7006;
  double t7011;
  double t6998;
  double t7012;
  double t7013;
  double t7016;
  double t7020;
  double t7031;
  double t7014;
  double t7032;
  double t7043;
  double t7048;
  double t7049;
  double t7051;
  double t7083;
  double t7087;
  double t7088;
  double t7092;
  double t7095;
  double t7097;
  double t7107;
  double t7096;
  double t7108;
  double t7115;
  double t7122;
  double t7125;
  double t7128;
  double t7121;
  double t7132;
  double t7137;
  double t7140;
  double t7147;
  double t7150;
  double t7187;
  double t7189;
  double t7195;
  double t7196;
  double t7203;
  double t7217;
  double t7218;
  double t7210;
  double t7219;
  double t7222;
  double t7229;
  double t7232;
  double t7233;
  double t7284;
  double t7285;
  double t7286;
  double t7289;
  double t7292;
  double t7302;
  double t7308;
  double t7293;
  double t7322;
  double t7347;
  double t7349;
  double t7354;
  double t5445;
  double t5461;
  double t7417;
  double t7429;
  double t7434;
  double t7442;
  double t7447;
  double t7451;
  double t7455;
  double t7457;
  double t7461;
  double t7475;
  double t7478;
  double t7479;
  double t7466;
  double t7480;
  double t7484;
  double t7492;
  double t7499;
  double t7501;
  double t7485;
  double t7502;
  double t7506;
  double t7514;
  double t7515;
  double t7521;
  double t7596;
  double t7598;
  double t7604;
  double t7586;
  double t7587;
  double t7593;
  double t7594;
  double t7605;
  double t7606;
  double t7612;
  double t7613;
  double t7615;
  double t7611;
  double t7616;
  double t7617;
  double t7620;
  double t7622;
  double t7623;
  double t7619;
  double t7630;
  double t7637;
  double t7639;
  double t7641;
  double t7642;
  double t7638;
  double t7644;
  double t7645;
  double t7653;
  double t7654;
  double t7656;
  double t7648;
  double t7664;
  double t7668;
  double t7670;
  double t7677;
  double t7678;
  double t7685;
  double t7687;
  double t7694;
  double t7702;
  double t7703;
  double t7706;
  double t7707;
  double t7708;
  double t7711;
  double t7714;
  double t7716;
  double t7718;
  double t7712;
  double t7729;
  double t7734;
  double t7736;
  double t7738;
  double t7743;
  double t7735;
  double t7747;
  double t7749;
  double t7755;
  double t7758;
  double t7760;
  double t7752;
  double t7763;
  double t7764;
  double t7766;
  double t7768;
  double t7769;
  double t7793;
  double t7799;
  double t7810;
  double t7811;
  double t7814;
  double t7817;
  double t7818;
  double t7815;
  double t7824;
  double t7832;
  double t7835;
  double t7836;
  double t7837;
  double t7834;
  double t7838;
  double t7841;
  double t7847;
  double t7849;
  double t7850;
  double t7873;
  double t7875;
  double t7876;
  double t7879;
  double t7884;
  double t7887;
  double t7888;
  double t7886;
  double t7889;
  double t7895;
  double t7904;
  double t7907;
  double t7908;
  double t7929;
  double t7930;
  double t7933;
  double t7935;
  double t7937;
  double t7940;
  double t7941;
  double t7938;
  double t7949;
  double t7962;
  double t7965;
  double t7967;
  double t8030;
  double t8031;
  double t8034;
  double t7982;
  double t7984;
  double t7986;
  double t7987;
  double t7989;
  double t7990;
  double t8029;
  double t8035;
  double t8039;
  double t8041;
  double t8044;
  double t8045;
  double t8040;
  double t8046;
  double t8048;
  double t8050;
  double t8053;
  double t8054;
  double t8049;
  double t8057;
  double t8058;
  double t8060;
  double t8061;
  double t8063;
  double t8059;
  double t8064;
  double t8067;
  double t8069;
  double t8070;
  double t8071;
  double t2408;
  double t2497;
  double t2501;
  double t2556;
  double t2569;
  double t2579;
  double t3067;
  double t3157;
  double t3183;
  double t3197;
  double t3200;
  double t3207;
  double t3512;
  double t3556;
  double t3558;
  double t3562;
  double t3573;
  double t3576;
  double t3833;
  double t3892;
  double t3893;
  double t3904;
  double t3906;
  double t3915;
  double t4228;
  double t4276;
  double t4283;
  double t4302;
  double t4315;
  double t4327;
  double t4555;
  double t4608;
  double t4613;
  double t4627;
  double t4645;
  double t4665;
  double t4893;
  double t4931;
  double t4935;
  double t4964;
  double t4968;
  double t4970;
  double t5093;
  double t5122;
  double t5145;
  double t5159;
  double t5163;
  double t5219;
  double t5324;
  double t5325;
  double t5337;
  double t5346;
  double t5422;
  double t5426;
  double t5789;
  double t5812;
  double t5816;
  double t5860;
  double t5884;
  double t5886;
  double t6114;
  double t6199;
  double t6210;
  double t6242;
  double t6250;
  double t6258;
  double t6476;
  double t6507;
  double t6512;
  double t6541;
  double t6544;
  double t6551;
  double t6833;
  double t6851;
  double t6856;
  double t6878;
  double t6880;
  double t6881;
  double t7045;
  double t7054;
  double t7055;
  double t7057;
  double t7058;
  double t7059;
  double t7139;
  double t7157;
  double t7160;
  double t7168;
  double t7171;
  double t7172;
  double t7227;
  double t7234;
  double t7236;
  double t7240;
  double t7252;
  double t7265;
  double t7315;
  double t7320;
  double t7324;
  double t7327;
  double t7355;
  double t7359;
  double t7369;
  double t7370;
  double t7381;
  double t7395;
  double t7401;
  double t5479;
  double t7512;
  double t7525;
  double t7543;
  double t7555;
  double t7568;
  double t7572;
  double t8280;
  double t8281;
  double t8282;
  double t7765;
  double t7772;
  double t7776;
  double t7780;
  double t7781;
  double t7782;
  double t7842;
  double t7853;
  double t7857;
  double t7859;
  double t7860;
  double t7866;
  double t7897;
  double t7910;
  double t7911;
  double t7919;
  double t7920;
  double t7921;
  double t7944;
  double t7947;
  double t7954;
  double t7955;
  double t7968;
  double t7969;
  double t7973;
  double t7976;
  double t8068;
  double t8074;
  double t8077;
  double t8081;
  double t8082;
  double t8083;
  t1541 = Cos(var1[3]);
  t1421 = Cos(var1[5]);
  t1544 = Sin(var1[4]);
  t1473 = Sin(var1[3]);
  t1562 = Sin(var1[5]);
  t1147 = Cos(var1[7]);
  t1176 = Cos(var1[6]);
  t1481 = -1.*t1421*t1473;
  t1569 = t1541*t1544*t1562;
  t1604 = t1481 + t1569;
  t1637 = t1176*t1604;
  t1690 = t1541*t1421*t1544;
  t1769 = t1473*t1562;
  t1824 = t1690 + t1769;
  t1852 = Sin(var1[6]);
  t1853 = t1824*t1852;
  t1900 = t1637 + t1853;
  t1911 = t1147*t1900;
  t1916 = Cos(var1[4]);
  t1962 = Sin(var1[7]);
  t1976 = -1.*t1541*t1916*t1962;
  t1987 = t1911 + t1976;
  t2066 = Cos(var1[9]);
  t881 = Cos(var1[8]);
  t995 = Sin(var1[9]);
  t2070 = Sin(var1[8]);
  t2119 = Cos(var1[10]);
  t2021 = -1.*t881*t995*t1987;
  t2083 = -1.*t2066*t1987*t2070;
  t2095 = t2021 + t2083;
  t859 = Sin(var1[10]);
  t2142 = t2066*t881*t1987;
  t2178 = -1.*t995*t1987*t2070;
  t2199 = t2142 + t2178;
  t2292 = Cos(var1[11]);
  t2109 = t859*t2095;
  t2218 = t2119*t2199;
  t2238 = t2109 + t2218;
  t808 = Sin(var1[11]);
  t2303 = t2119*t2095;
  t2310 = -1.*t859*t2199;
  t2330 = t2303 + t2310;
  t2468 = Cos(var1[12]);
  t2268 = -1.*t808*t2238;
  t2369 = t2292*t2330;
  t2385 = t2268 + t2369;
  t19 = Sin(var1[12]);
  t2473 = t2292*t2238;
  t2486 = t808*t2330;
  t2489 = t2473 + t2486;
  t2652 = t1421*t1473;
  t2670 = -1.*t1541*t1544*t1562;
  t2740 = t2652 + t2670;
  t2773 = t1176*t2740;
  t2820 = -1.*t1824*t1852;
  t2842 = t2773 + t2820;
  t2615 = t1176*t1824;
  t2749 = t2740*t1852;
  t2755 = t2615 + t2749;
  t2760 = t881*t2755*t1962;
  t2849 = t2842*t2070;
  t2856 = t2760 + t2849;
  t2880 = t881*t2842;
  t2881 = -1.*t2755*t1962*t2070;
  t2897 = t2880 + t2881;
  t2875 = -1.*t995*t2856;
  t2903 = t2066*t2897;
  t2930 = t2875 + t2903;
  t2952 = t2066*t2856;
  t2957 = t995*t2897;
  t2972 = t2952 + t2957;
  t2947 = t859*t2930;
  t2974 = t2119*t2972;
  t2975 = t2947 + t2974;
  t2990 = t2119*t2930;
  t2998 = -1.*t859*t2972;
  t3003 = t2990 + t2998;
  t2986 = -1.*t808*t2975;
  t3017 = t2292*t3003;
  t3057 = t2986 + t3017;
  t3112 = t2292*t2975;
  t3131 = t808*t3003;
  t3156 = t3112 + t3131;
  t3311 = -1.*t1176*t1604;
  t3316 = t3311 + t2820;
  t3282 = -1.*t1604*t1852;
  t3284 = t2615 + t3282;
  t3294 = t881*t3284*t1962;
  t3317 = t3316*t2070;
  t3319 = t3294 + t3317;
  t3322 = t881*t3316;
  t3346 = -1.*t3284*t1962*t2070;
  t3359 = t3322 + t3346;
  t3320 = -1.*t995*t3319;
  t3365 = t2066*t3359;
  t3374 = t3320 + t3365;
  t3399 = t2066*t3319;
  t3407 = t995*t3359;
  t3423 = t3399 + t3407;
  t3381 = t859*t3374;
  t3428 = t2119*t3423;
  t3452 = t3381 + t3428;
  t3487 = t2119*t3374;
  t3489 = -1.*t859*t3423;
  t3496 = t3487 + t3489;
  t3476 = -1.*t808*t3452;
  t3501 = t2292*t3496;
  t3511 = t3476 + t3501;
  t3520 = t2292*t3452;
  t3526 = t808*t3496;
  t3529 = t3520 + t3526;
  t3666 = t1541*t1916*t1421*t1176;
  t3667 = -1.*t1541*t1916*t1562*t1852;
  t3669 = t3666 + t3667;
  t3598 = -1.*t1541*t1147*t1544;
  t3614 = t1541*t1916*t1176*t1562;
  t3636 = t1541*t1916*t1421*t1852;
  t3640 = t3614 + t3636;
  t3641 = t3640*t1962;
  t3648 = t3598 + t3641;
  t3662 = t881*t3648;
  t3675 = t3669*t2070;
  t3678 = t3662 + t3675;
  t3710 = t881*t3669;
  t3717 = -1.*t3648*t2070;
  t3725 = t3710 + t3717;
  t3707 = -1.*t995*t3678;
  t3743 = t2066*t3725;
  t3785 = t3707 + t3743;
  t3791 = t2066*t3678;
  t3793 = t995*t3725;
  t3795 = t3791 + t3793;
  t3787 = t859*t3785;
  t3804 = t2119*t3795;
  t3811 = t3787 + t3804;
  t3821 = t2119*t3785;
  t3823 = -1.*t859*t3795;
  t3825 = t3821 + t3823;
  t3813 = -1.*t808*t3811;
  t3831 = t2292*t3825;
  t3832 = t3813 + t3831;
  t3848 = t2292*t3811;
  t3868 = t808*t3825;
  t3887 = t3848 + t3868;
  t4000 = -1.*t1421*t1473*t1544;
  t4012 = t1541*t1562;
  t4044 = t4000 + t4012;
  t3949 = -1.*t1541*t1421;
  t3962 = -1.*t1473*t1544*t1562;
  t3993 = t3949 + t3962;
  t4076 = t1176*t4044;
  t4077 = -1.*t3993*t1852;
  t4099 = t4076 + t4077;
  t3931 = -1.*t1916*t1147*t1473;
  t3998 = t1176*t3993;
  t4051 = t4044*t1852;
  t4058 = t3998 + t4051;
  t4059 = t4058*t1962;
  t4060 = t3931 + t4059;
  t4065 = t881*t4060;
  t4103 = t4099*t2070;
  t4106 = t4065 + t4103;
  t4110 = t881*t4099;
  t4115 = -1.*t4060*t2070;
  t4118 = t4110 + t4115;
  t4109 = -1.*t995*t4106;
  t4119 = t2066*t4118;
  t4125 = t4109 + t4119;
  t4137 = t2066*t4106;
  t4147 = t995*t4118;
  t4152 = t4137 + t4147;
  t4134 = t859*t4125;
  t4168 = t2119*t4152;
  t4184 = t4134 + t4168;
  t4188 = t2119*t4125;
  t4201 = -1.*t859*t4152;
  t4207 = t4188 + t4201;
  t4187 = -1.*t808*t4184;
  t4209 = t2292*t4207;
  t4216 = t4187 + t4209;
  t4260 = t2292*t4184;
  t4264 = t808*t4207;
  t4265 = t4260 + t4264;
  t4382 = t1541*t1916*t1147;
  t4383 = t1900*t1962;
  t4390 = t4382 + t4383;
  t4405 = -1.*t881*t4390;
  t4424 = -1.*t3284*t2070;
  t4429 = t4405 + t4424;
  t4440 = t881*t3284;
  t4445 = -1.*t4390*t2070;
  t4472 = t4440 + t4445;
  t4437 = t995*t4429;
  t4481 = t2066*t4472;
  t4483 = t4437 + t4481;
  t4501 = t2066*t4429;
  t4502 = -1.*t995*t4472;
  t4528 = t4501 + t4502;
  t4487 = -1.*t859*t4483;
  t4531 = t2119*t4528;
  t4535 = t4487 + t4531;
  t4537 = t2119*t4483;
  t4538 = t859*t4528;
  t4542 = t4537 + t4538;
  t4536 = t808*t4535;
  t4546 = t2292*t4542;
  t4550 = t4536 + t4546;
  t4564 = t2292*t4535;
  t4593 = -1.*t808*t4542;
  t4595 = t4564 + t4593;
  t4692 = t881*t4390;
  t4703 = t3284*t2070;
  t4719 = t4692 + t4703;
  t4734 = -1.*t995*t4719;
  t4735 = t4734 + t4481;
  t4748 = -1.*t2066*t4719;
  t4788 = t4748 + t4502;
  t4745 = -1.*t859*t4735;
  t4790 = t2119*t4788;
  t4827 = t4745 + t4790;
  t4830 = t2119*t4735;
  t4853 = t859*t4788;
  t4869 = t4830 + t4853;
  t4829 = t808*t4827;
  t4891 = t2292*t4869;
  t4892 = t4829 + t4891;
  t4896 = t2292*t4827;
  t4921 = -1.*t808*t4869;
  t4922 = t4896 + t4921;
  t4988 = t2066*t4719;
  t4990 = t995*t4472;
  t4991 = t4988 + t4990;
  t5001 = -1.*t2119*t4991;
  t5002 = t4745 + t5001;
  t5049 = -1.*t859*t4991;
  t5052 = t4830 + t5049;
  t5012 = t808*t5002;
  t5065 = t2292*t5052;
  t5080 = t5012 + t5065;
  t5097 = t2292*t5002;
  t5098 = -1.*t808*t5052;
  t5101 = t5097 + t5098;
  t5248 = t859*t4735;
  t5256 = t2119*t4991;
  t5259 = t5248 + t5256;
  t5261 = -1.*t808*t5259;
  t5280 = t5261 + t5065;
  t5316 = -1.*t2292*t5259;
  t5317 = t5316 + t5098;
  t5314 = -1.*t19*t5280;
  t5332 = t2468*t5280;
  t5373 = t2292*t5259;
  t5374 = t808*t5052;
  t5402 = t5373 + t5374;
  t5508 = t1541*t1421;
  t5509 = t1473*t1544*t1562;
  t5513 = t5508 + t5509;
  t5532 = t1176*t5513;
  t5535 = t1421*t1473*t1544;
  t5546 = -1.*t1541*t1562;
  t5553 = t5535 + t5546;
  t5554 = t5553*t1852;
  t5555 = t5532 + t5554;
  t5559 = t1147*t5555;
  t5564 = -1.*t1916*t1473*t1962;
  t5566 = t5559 + t5564;
  t5579 = -1.*t881*t995*t5566;
  t5619 = -1.*t2066*t5566*t2070;
  t5650 = t5579 + t5619;
  t5673 = t2066*t881*t5566;
  t5674 = -1.*t995*t5566*t2070;
  t5682 = t5673 + t5674;
  t5668 = t859*t5650;
  t5690 = t2119*t5682;
  t5692 = t5668 + t5690;
  t5737 = t2119*t5650;
  t5760 = -1.*t859*t5682;
  t5782 = t5737 + t5760;
  t5709 = -1.*t808*t5692;
  t5786 = t2292*t5782;
  t5787 = t5709 + t5786;
  t5790 = t2292*t5692;
  t5794 = t808*t5782;
  t5804 = t5790 + t5794;
  t5949 = -1.*t5553*t1852;
  t5954 = t3998 + t5949;
  t5926 = t1176*t5553;
  t5929 = t3993*t1852;
  t5947 = t5926 + t5929;
  t5948 = t881*t5947*t1962;
  t5957 = t5954*t2070;
  t5973 = t5948 + t5957;
  t5980 = t881*t5954;
  t6011 = -1.*t5947*t1962*t2070;
  t6015 = t5980 + t6011;
  t5978 = -1.*t995*t5973;
  t6017 = t2066*t6015;
  t6021 = t5978 + t6017;
  t6031 = t2066*t5973;
  t6033 = t995*t6015;
  t6039 = t6031 + t6033;
  t6027 = t859*t6021;
  t6042 = t2119*t6039;
  t6044 = t6027 + t6042;
  t6062 = t2119*t6021;
  t6065 = -1.*t859*t6039;
  t6084 = t6062 + t6065;
  t6059 = -1.*t808*t6044;
  t6088 = t2292*t6084;
  t6102 = t6059 + t6088;
  t6167 = t2292*t6044;
  t6172 = t808*t6084;
  t6178 = t6167 + t6172;
  t6321 = -1.*t1176*t5513;
  t6327 = t6321 + t5949;
  t6300 = -1.*t5513*t1852;
  t6309 = t5926 + t6300;
  t6316 = t881*t6309*t1962;
  t6329 = t6327*t2070;
  t6340 = t6316 + t6329;
  t6354 = t881*t6327;
  t6359 = -1.*t6309*t1962*t2070;
  t6360 = t6354 + t6359;
  t6341 = -1.*t995*t6340;
  t6362 = t2066*t6360;
  t6364 = t6341 + t6362;
  t6394 = t2066*t6340;
  t6398 = t995*t6360;
  t6406 = t6394 + t6398;
  t6389 = t859*t6364;
  t6410 = t2119*t6406;
  t6411 = t6389 + t6410;
  t6429 = t2119*t6364;
  t6430 = -1.*t859*t6406;
  t6457 = t6429 + t6430;
  t6428 = -1.*t808*t6411;
  t6467 = t2292*t6457;
  t6470 = t6428 + t6467;
  t6483 = t2292*t6411;
  t6487 = t808*t6457;
  t6495 = t6483 + t6487;
  t6660 = t1916*t1421*t1176*t1473;
  t6672 = -1.*t1916*t1473*t1562*t1852;
  t6684 = t6660 + t6672;
  t6574 = -1.*t1147*t1473*t1544;
  t6593 = t1916*t1176*t1473*t1562;
  t6612 = t1916*t1421*t1473*t1852;
  t6616 = t6593 + t6612;
  t6632 = t6616*t1962;
  t6646 = t6574 + t6632;
  t6647 = t881*t6646;
  t6687 = t6684*t2070;
  t6691 = t6647 + t6687;
  t6697 = t881*t6684;
  t6698 = -1.*t6646*t2070;
  t6700 = t6697 + t6698;
  t6693 = -1.*t995*t6691;
  t6709 = t2066*t6700;
  t6730 = t6693 + t6709;
  t6744 = t2066*t6691;
  t6748 = t995*t6700;
  t6756 = t6744 + t6748;
  t6731 = t859*t6730;
  t6757 = t2119*t6756;
  t6758 = t6731 + t6757;
  t6761 = t2119*t6730;
  t6770 = -1.*t859*t6756;
  t6784 = t6761 + t6770;
  t6760 = -1.*t808*t6758;
  t6789 = t2292*t6784;
  t6790 = t6760 + t6789;
  t6839 = t2292*t6758;
  t6843 = t808*t6784;
  t6845 = t6839 + t6843;
  t6900 = t1916*t1147*t1473;
  t6901 = t5555*t1962;
  t6905 = t6900 + t6901;
  t6906 = -1.*t881*t6905;
  t6913 = -1.*t6309*t2070;
  t6914 = t6906 + t6913;
  t6932 = t881*t6309;
  t6933 = -1.*t6905*t2070;
  t6947 = t6932 + t6933;
  t6915 = t995*t6914;
  t6980 = t2066*t6947;
  t6988 = t6915 + t6980;
  t6999 = t2066*t6914;
  t7006 = -1.*t995*t6947;
  t7011 = t6999 + t7006;
  t6998 = -1.*t859*t6988;
  t7012 = t2119*t7011;
  t7013 = t6998 + t7012;
  t7016 = t2119*t6988;
  t7020 = t859*t7011;
  t7031 = t7016 + t7020;
  t7014 = t808*t7013;
  t7032 = t2292*t7031;
  t7043 = t7014 + t7032;
  t7048 = t2292*t7013;
  t7049 = -1.*t808*t7031;
  t7051 = t7048 + t7049;
  t7083 = t881*t6905;
  t7087 = t6309*t2070;
  t7088 = t7083 + t7087;
  t7092 = -1.*t995*t7088;
  t7095 = t7092 + t6980;
  t7097 = -1.*t2066*t7088;
  t7107 = t7097 + t7006;
  t7096 = -1.*t859*t7095;
  t7108 = t2119*t7107;
  t7115 = t7096 + t7108;
  t7122 = t2119*t7095;
  t7125 = t859*t7107;
  t7128 = t7122 + t7125;
  t7121 = t808*t7115;
  t7132 = t2292*t7128;
  t7137 = t7121 + t7132;
  t7140 = t2292*t7115;
  t7147 = -1.*t808*t7128;
  t7150 = t7140 + t7147;
  t7187 = t2066*t7088;
  t7189 = t995*t6947;
  t7195 = t7187 + t7189;
  t7196 = -1.*t2119*t7195;
  t7203 = t7096 + t7196;
  t7217 = -1.*t859*t7195;
  t7218 = t7122 + t7217;
  t7210 = t808*t7203;
  t7219 = t2292*t7218;
  t7222 = t7210 + t7219;
  t7229 = t2292*t7203;
  t7232 = -1.*t808*t7218;
  t7233 = t7229 + t7232;
  t7284 = t859*t7095;
  t7285 = t2119*t7195;
  t7286 = t7284 + t7285;
  t7289 = -1.*t808*t7286;
  t7292 = t7289 + t7219;
  t7302 = -1.*t2292*t7286;
  t7308 = t7302 + t7232;
  t7293 = -1.*t19*t7292;
  t7322 = t2468*t7292;
  t7347 = t2292*t7286;
  t7349 = t808*t7218;
  t7354 = t7347 + t7349;
  t5445 = -1.*t19*t5402;
  t5461 = t5332 + t5445;
  t7417 = t1916*t1176*t1562;
  t7429 = t1916*t1421*t1852;
  t7434 = t7417 + t7429;
  t7442 = t1147*t7434;
  t7447 = t1544*t1962;
  t7451 = t7442 + t7447;
  t7455 = -1.*t881*t995*t7451;
  t7457 = -1.*t2066*t7451*t2070;
  t7461 = t7455 + t7457;
  t7475 = t2066*t881*t7451;
  t7478 = -1.*t995*t7451*t2070;
  t7479 = t7475 + t7478;
  t7466 = t859*t7461;
  t7480 = t2119*t7479;
  t7484 = t7466 + t7480;
  t7492 = t2119*t7461;
  t7499 = -1.*t859*t7479;
  t7501 = t7492 + t7499;
  t7485 = -1.*t808*t7484;
  t7502 = t2292*t7501;
  t7506 = t7485 + t7502;
  t7514 = t2292*t7484;
  t7515 = t808*t7501;
  t7521 = t7514 + t7515;
  t7596 = -1.*t1916*t1176*t1562;
  t7598 = -1.*t1916*t1421*t1852;
  t7604 = t7596 + t7598;
  t7586 = t1916*t1421*t1176;
  t7587 = -1.*t1916*t1562*t1852;
  t7593 = t7586 + t7587;
  t7594 = t881*t7593*t1962;
  t7605 = t7604*t2070;
  t7606 = t7594 + t7605;
  t7612 = t881*t7604;
  t7613 = -1.*t7593*t1962*t2070;
  t7615 = t7612 + t7613;
  t7611 = -1.*t995*t7606;
  t7616 = t2066*t7615;
  t7617 = t7611 + t7616;
  t7620 = t2066*t7606;
  t7622 = t995*t7615;
  t7623 = t7620 + t7622;
  t7619 = t859*t7617;
  t7630 = t2119*t7623;
  t7637 = t7619 + t7630;
  t7639 = t2119*t7617;
  t7641 = -1.*t859*t7623;
  t7642 = t7639 + t7641;
  t7638 = -1.*t808*t7637;
  t7644 = t2292*t7642;
  t7645 = t7638 + t7644;
  t7653 = t2292*t7637;
  t7654 = t808*t7642;
  t7656 = t7653 + t7654;
  t7648 = t19*t7645;
  t7664 = t2468*t7656;
  t7668 = t7648 + t7664;
  t7670 = 0.642788*t7668;
  t7677 = t2468*t7645;
  t7678 = -1.*t19*t7656;
  t7685 = t7677 + t7678;
  t7687 = 0.766044*t7685;
  t7694 = t7670 + t7687;
  t7702 = -1.*t1147*t1544;
  t7703 = t7434*t1962;
  t7706 = t7702 + t7703;
  t7707 = -1.*t881*t7706;
  t7708 = -1.*t7593*t2070;
  t7711 = t7707 + t7708;
  t7714 = t881*t7593;
  t7716 = -1.*t7706*t2070;
  t7718 = t7714 + t7716;
  t7712 = t995*t7711;
  t7729 = t2066*t7718;
  t7734 = t7712 + t7729;
  t7736 = t2066*t7711;
  t7738 = -1.*t995*t7718;
  t7743 = t7736 + t7738;
  t7735 = -1.*t859*t7734;
  t7747 = t2119*t7743;
  t7749 = t7735 + t7747;
  t7755 = t2119*t7734;
  t7758 = t859*t7743;
  t7760 = t7755 + t7758;
  t7752 = t808*t7749;
  t7763 = t2292*t7760;
  t7764 = t7752 + t7763;
  t7766 = t2292*t7749;
  t7768 = -1.*t808*t7760;
  t7769 = t7766 + t7768;
  t7793 = t881*t7706;
  t7799 = t7593*t2070;
  t7810 = t7793 + t7799;
  t7811 = -1.*t995*t7810;
  t7814 = t7811 + t7729;
  t7817 = -1.*t2066*t7810;
  t7818 = t7817 + t7738;
  t7815 = -1.*t859*t7814;
  t7824 = t2119*t7818;
  t7832 = t7815 + t7824;
  t7835 = t2119*t7814;
  t7836 = t859*t7818;
  t7837 = t7835 + t7836;
  t7834 = t808*t7832;
  t7838 = t2292*t7837;
  t7841 = t7834 + t7838;
  t7847 = t2292*t7832;
  t7849 = -1.*t808*t7837;
  t7850 = t7847 + t7849;
  t7873 = t2066*t7810;
  t7875 = t995*t7718;
  t7876 = t7873 + t7875;
  t7879 = -1.*t2119*t7876;
  t7884 = t7815 + t7879;
  t7887 = -1.*t859*t7876;
  t7888 = t7835 + t7887;
  t7886 = t808*t7884;
  t7889 = t2292*t7888;
  t7895 = t7886 + t7889;
  t7904 = t2292*t7884;
  t7907 = -1.*t808*t7888;
  t7908 = t7904 + t7907;
  t7929 = t859*t7814;
  t7930 = t2119*t7876;
  t7933 = t7929 + t7930;
  t7935 = -1.*t808*t7933;
  t7937 = t7935 + t7889;
  t7940 = -1.*t2292*t7933;
  t7941 = t7940 + t7907;
  t7938 = -1.*t19*t7937;
  t7949 = t2468*t7937;
  t7962 = t2292*t7933;
  t7965 = t808*t7888;
  t7967 = t7962 + t7965;
  t8030 = -1.*t1421*t1176*t1544;
  t8031 = t1544*t1562*t1852;
  t8034 = t8030 + t8031;
  t7982 = -1.*t1916*t1147;
  t7984 = -1.*t1176*t1544*t1562;
  t7986 = -1.*t1421*t1544*t1852;
  t7987 = t7984 + t7986;
  t7989 = t7987*t1962;
  t7990 = t7982 + t7989;
  t8029 = t881*t7990;
  t8035 = t8034*t2070;
  t8039 = t8029 + t8035;
  t8041 = t881*t8034;
  t8044 = -1.*t7990*t2070;
  t8045 = t8041 + t8044;
  t8040 = -1.*t995*t8039;
  t8046 = t2066*t8045;
  t8048 = t8040 + t8046;
  t8050 = t2066*t8039;
  t8053 = t995*t8045;
  t8054 = t8050 + t8053;
  t8049 = t859*t8048;
  t8057 = t2119*t8054;
  t8058 = t8049 + t8057;
  t8060 = t2119*t8048;
  t8061 = -1.*t859*t8054;
  t8063 = t8060 + t8061;
  t8059 = -1.*t808*t8058;
  t8064 = t2292*t8063;
  t8067 = t8059 + t8064;
  t8069 = t2292*t8058;
  t8070 = t808*t8063;
  t8071 = t8069 + t8070;
  t2408 = t19*t2385;
  t2497 = t2468*t2489;
  t2501 = t2408 + t2497;
  t2556 = t2468*t2385;
  t2569 = -1.*t19*t2489;
  t2579 = t2556 + t2569;
  t3067 = t19*t3057;
  t3157 = t2468*t3156;
  t3183 = t3067 + t3157;
  t3197 = t2468*t3057;
  t3200 = -1.*t19*t3156;
  t3207 = t3197 + t3200;
  t3512 = t19*t3511;
  t3556 = t2468*t3529;
  t3558 = t3512 + t3556;
  t3562 = t2468*t3511;
  t3573 = -1.*t19*t3529;
  t3576 = t3562 + t3573;
  t3833 = t19*t3832;
  t3892 = t2468*t3887;
  t3893 = t3833 + t3892;
  t3904 = t2468*t3832;
  t3906 = -1.*t19*t3887;
  t3915 = t3904 + t3906;
  t4228 = t19*t4216;
  t4276 = t2468*t4265;
  t4283 = t4228 + t4276;
  t4302 = t2468*t4216;
  t4315 = -1.*t19*t4265;
  t4327 = t4302 + t4315;
  t4555 = -1.*t19*t4550;
  t4608 = t2468*t4595;
  t4613 = t4555 + t4608;
  t4627 = t2468*t4550;
  t4645 = t19*t4595;
  t4665 = t4627 + t4645;
  t4893 = -1.*t19*t4892;
  t4931 = t2468*t4922;
  t4935 = t4893 + t4931;
  t4964 = t2468*t4892;
  t4968 = t19*t4922;
  t4970 = t4964 + t4968;
  t5093 = -1.*t19*t5080;
  t5122 = t2468*t5101;
  t5145 = t5093 + t5122;
  t5159 = t2468*t5080;
  t5163 = t19*t5101;
  t5219 = t5159 + t5163;
  t5324 = t2468*t5317;
  t5325 = t5314 + t5324;
  t5337 = t19*t5317;
  t5346 = t5332 + t5337;
  t5422 = -1.*t2468*t5402;
  t5426 = t5314 + t5422;
  t5789 = t19*t5787;
  t5812 = t2468*t5804;
  t5816 = t5789 + t5812;
  t5860 = t2468*t5787;
  t5884 = -1.*t19*t5804;
  t5886 = t5860 + t5884;
  t6114 = t19*t6102;
  t6199 = t2468*t6178;
  t6210 = t6114 + t6199;
  t6242 = t2468*t6102;
  t6250 = -1.*t19*t6178;
  t6258 = t6242 + t6250;
  t6476 = t19*t6470;
  t6507 = t2468*t6495;
  t6512 = t6476 + t6507;
  t6541 = t2468*t6470;
  t6544 = -1.*t19*t6495;
  t6551 = t6541 + t6544;
  t6833 = t19*t6790;
  t6851 = t2468*t6845;
  t6856 = t6833 + t6851;
  t6878 = t2468*t6790;
  t6880 = -1.*t19*t6845;
  t6881 = t6878 + t6880;
  t7045 = -1.*t19*t7043;
  t7054 = t2468*t7051;
  t7055 = t7045 + t7054;
  t7057 = t2468*t7043;
  t7058 = t19*t7051;
  t7059 = t7057 + t7058;
  t7139 = -1.*t19*t7137;
  t7157 = t2468*t7150;
  t7160 = t7139 + t7157;
  t7168 = t2468*t7137;
  t7171 = t19*t7150;
  t7172 = t7168 + t7171;
  t7227 = -1.*t19*t7222;
  t7234 = t2468*t7233;
  t7236 = t7227 + t7234;
  t7240 = t2468*t7222;
  t7252 = t19*t7233;
  t7265 = t7240 + t7252;
  t7315 = t2468*t7308;
  t7320 = t7293 + t7315;
  t7324 = t19*t7308;
  t7327 = t7322 + t7324;
  t7355 = -1.*t2468*t7354;
  t7359 = t7293 + t7355;
  t7369 = -1.*t19*t7354;
  t7370 = t7322 + t7369;
  t7381 = t19*t5280;
  t7395 = t2468*t5402;
  t7401 = t7381 + t7395;
  t5479 = 0.642788*t5461;
  t7512 = t19*t7506;
  t7525 = t2468*t7521;
  t7543 = t7512 + t7525;
  t7555 = t2468*t7506;
  t7568 = -1.*t19*t7521;
  t7572 = t7555 + t7568;
  t8280 = -0.766044*t7668;
  t8281 = 0.642788*t7685;
  t8282 = t8280 + t8281;
  t7765 = -1.*t19*t7764;
  t7772 = t2468*t7769;
  t7776 = t7765 + t7772;
  t7780 = t2468*t7764;
  t7781 = t19*t7769;
  t7782 = t7780 + t7781;
  t7842 = -1.*t19*t7841;
  t7853 = t2468*t7850;
  t7857 = t7842 + t7853;
  t7859 = t2468*t7841;
  t7860 = t19*t7850;
  t7866 = t7859 + t7860;
  t7897 = -1.*t19*t7895;
  t7910 = t2468*t7908;
  t7911 = t7897 + t7910;
  t7919 = t2468*t7895;
  t7920 = t19*t7908;
  t7921 = t7919 + t7920;
  t7944 = t2468*t7941;
  t7947 = t7938 + t7944;
  t7954 = t19*t7941;
  t7955 = t7949 + t7954;
  t7968 = -1.*t2468*t7967;
  t7969 = t7938 + t7968;
  t7973 = -1.*t19*t7967;
  t7976 = t7949 + t7973;
  t8068 = t19*t8067;
  t8074 = t2468*t8071;
  t8077 = t8068 + t8074;
  t8081 = t2468*t8067;
  t8082 = -1.*t19*t8071;
  t8083 = t8081 + t8082;
  p_output1[0]=(0.642788*t4283 + 0.766044*t4327)*var2[3] + (0.642788*t3893 + 0.766044*t3915)*var2[4] + (0.642788*t3183 + 0.766044*t3207)*var2[5] + (0.642788*t3558 + 0.766044*t3576)*var2[6] + (0.642788*t2501 + 0.766044*t2579)*var2[7] + (0.766044*t4613 + 0.642788*t4665)*var2[8] + (0.766044*t4935 + 0.642788*t4970)*var2[9] + (0.766044*t5145 + 0.642788*t5219)*var2[10] + (0.766044*t5325 + 0.642788*t5346)*var2[11] + (0.766044*t5426 + t5479)*var2[12];
  p_output1[1]=(0.766044*t5461 + 0.642788*t7401)*var2[3] + (0.642788*t6856 + 0.766044*t6881)*var2[4] + (0.642788*t6210 + 0.766044*t6258)*var2[5] + (0.642788*t6512 + 0.766044*t6551)*var2[6] + (0.642788*t5816 + 0.766044*t5886)*var2[7] + (0.766044*t7055 + 0.642788*t7059)*var2[8] + (0.766044*t7160 + 0.642788*t7172)*var2[9] + (0.766044*t7236 + 0.642788*t7265)*var2[10] + (0.766044*t7320 + 0.642788*t7327)*var2[11] + (0.766044*t7359 + 0.642788*t7370)*var2[12];
  p_output1[2]=(0.642788*t8077 + 0.766044*t8083)*var2[4] + t7694*var2[5] + t7694*var2[6] + (0.642788*t7543 + 0.766044*t7572)*var2[7] + (0.766044*t7776 + 0.642788*t7782)*var2[8] + (0.766044*t7857 + 0.642788*t7866)*var2[9] + (0.766044*t7911 + 0.642788*t7921)*var2[10] + (0.766044*t7947 + 0.642788*t7955)*var2[11] + (0.766044*t7969 + 0.642788*t7976)*var2[12];
  p_output1[3]=(t1473*t1916*t1962 + t1147*t4058)*var2[3] + (t1541*t1544*t1962 + t1147*t3640)*var2[4] + t1147*t2755*var2[5] + t1147*t3284*var2[6] + (-1.*t1147*t1541*t1916 - 1.*t1900*t1962)*var2[7];
  p_output1[4]=t1987*var2[3] + (t1473*t1544*t1962 + t1147*t6616)*var2[4] + t1147*t5947*var2[5] + t1147*t6309*var2[6] + (t3931 - 1.*t1962*t5555)*var2[7];
  p_output1[5]=(t1916*t1962 + t1147*t7987)*var2[4] + t1147*t7593*var2[5] + t1147*t7593*var2[6] + (t1147*t1544 - 1.*t1962*t7434)*var2[7];
  p_output1[6]=(-0.766044*t4283 + 0.642788*t4327)*var2[3] + (-0.766044*t3893 + 0.642788*t3915)*var2[4] + (-0.766044*t3183 + 0.642788*t3207)*var2[5] + (-0.766044*t3558 + 0.642788*t3576)*var2[6] + (-0.766044*t2501 + 0.642788*t2579)*var2[7] + (0.642788*t4613 - 0.766044*t4665)*var2[8] + (0.642788*t4935 - 0.766044*t4970)*var2[9] + (0.642788*t5145 - 0.766044*t5219)*var2[10] + (0.642788*t5325 - 0.766044*t5346)*var2[11] + (0.642788*t5426 - 0.766044*t5461)*var2[12];
  p_output1[7]=(t5479 - 0.766044*t7401)*var2[3] + (-0.766044*t6856 + 0.642788*t6881)*var2[4] + (-0.766044*t6210 + 0.642788*t6258)*var2[5] + (-0.766044*t6512 + 0.642788*t6551)*var2[6] + (-0.766044*t5816 + 0.642788*t5886)*var2[7] + (0.642788*t7055 - 0.766044*t7059)*var2[8] + (0.642788*t7160 - 0.766044*t7172)*var2[9] + (0.642788*t7236 - 0.766044*t7265)*var2[10] + (0.642788*t7320 - 0.766044*t7327)*var2[11] + (0.642788*t7359 - 0.766044*t7370)*var2[12];
  p_output1[8]=(-0.766044*t8077 + 0.642788*t8083)*var2[4] + t8282*var2[5] + t8282*var2[6] + (-0.766044*t7543 + 0.642788*t7572)*var2[7] + (0.642788*t7776 - 0.766044*t7782)*var2[8] + (0.642788*t7857 - 0.766044*t7866)*var2[9] + (0.642788*t7911 - 0.766044*t7921)*var2[10] + (0.642788*t7947 - 0.766044*t7955)*var2[11] + (0.642788*t7969 - 0.766044*t7976)*var2[12];
}



#ifdef MATLAB_MEX_FILE

#include "mex.h"
/*
 * Main function
 */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  size_t mrows, ncols;

  double *var1,*var2;
  double *p_output1;

  /*  Check for proper number of arguments.  */ 
  if( nrhs != 2)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:invalidNumInputs", "Two input(s) required (var1,var2).");
    }
  else if( nlhs > 1)
    {
      mexErrMsgIdAndTxt("MATLAB:MShaped:maxlhs", "Too many output arguments.");
    }

  /*  The input must be a noncomplex double vector or scaler.  */
  mrows = mxGetM(prhs[0]);
  ncols = mxGetN(prhs[0]);
  if( !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) ||
    ( !(mrows == 20 && ncols == 1) && 
      !(mrows == 1 && ncols == 20))) 
    {
      mexErrMsgIdAndTxt( "MATLAB:MShaped:inputNotRealVector", "var1 is wrong.");
    }
  mrows = mxGetM(prhs[1]);
  ncols = mxGetN(prhs[1]);
  if( !mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) ||
    ( !(mrows == 20 && ncols == 1) && 
      !(mrows == 1 && ncols == 20))) 
    {
      mexErrMsgIdAndTxt( "MATLAB:MShaped:inputNotRealVector", "var2 is wrong.");
    }

  /*  Assign pointers to each input.  */
  var1 = mxGetPr(prhs[0]);
  var2 = mxGetPr(prhs[1]);
   


   
  /*  Create matrices for return arguments.  */
  plhs[0] = mxCreateDoubleMatrix((mwSize) 3, (mwSize) 3, mxREAL);
  p_output1 = mxGetPr(plhs[0]);


  /* Call the calculation subroutine. */
  output1(p_output1,var1,var2);


}

#else // MATLAB_MEX_FILE

#include "dR_LeftToeBottom_mex.hh"

namespace SymExpression
{

void dR_LeftToeBottom_mex_raw(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}

}

#endif // MATLAB_MEX_FILE

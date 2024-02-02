/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:26:52 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jb_LeftToe_src.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE inline        /* use standard inline */
#endif

/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
INLINE double Power(double x, double y) { return pow(x, y); }
INLINE double Sqrt(double x) { return sqrt(x); }

INLINE double Abs(double x) { return fabs(x); }

INLINE double Exp(double x) { return exp(x); }
INLINE double Log(double x) { return log(x); }

INLINE double Sin(double x) { return sin(x); }
INLINE double Cos(double x) { return cos(x); }
INLINE double Tan(double x) { return tan(x); }

INLINE double Csc(double x) { return 1.0/sin(x); }
INLINE double Sec(double x) { return 1.0/cos(x); }

INLINE double ArcSin(double x) { return asin(x); }
INLINE double ArcCos(double x) { return acos(x); }
//INLINE double ArcTan(double x) { return atan(x); }

/* update ArcTan function to use atan2 instead. */
INLINE double ArcTan(double x, double y) { return atan2(y,x); }

INLINE double Sinh(double x) { return sinh(x); }
INLINE double Cosh(double x) { return cosh(x); }
INLINE double Tanh(double x) { return tanh(x); }

#define E 2.71828182845904523536029
#define Pi 3.14159265358979323846264
#define Degree 0.01745329251994329576924

/*
 * Sub functions
 */
static void output1(double *p_output1,const double *var1)
{
  double t217;
  double t188;
  double t194;
  double t225;
  double t271;
  double t204;
  double t229;
  double t248;
  double t163;
  double t277;
  double t292;
  double t312;
  double t369;
  double t263;
  double t315;
  double t317;
  double t127;
  double t383;
  double t390;
  double t399;
  double t108;
  double t459;
  double t485;
  double t504;
  double t355;
  double t443;
  double t446;
  double t516;
  double t547;
  double t451;
  double t520;
  double t529;
  double t89;
  double t567;
  double t569;
  double t575;
  double t586;
  double t606;
  double t619;
  double t665;
  double t541;
  double t633;
  double t639;
  double t88;
  double t670;
  double t673;
  double t674;
  double t32;
  double t792;
  double t797;
  double t799;
  double t783;
  double t802;
  double t804;
  double t816;
  double t818;
  double t831;
  double t840;
  double t854;
  double t810;
  double t819;
  double t823;
  double t829;
  double t861;
  double t878;
  double t898;
  double t904;
  double t905;
  double t908;
  double t912;
  double t705;
  double t708;
  double t711;
  double t732;
  double t883;
  double t916;
  double t939;
  double t954;
  double t957;
  double t958;
  double t1024;
  double t1027;
  double t1040;
  double t1061;
  double t1064;
  double t1065;
  double t643;
  double t683;
  double t690;
  double t714;
  double t719;
  double t724;
  double t737;
  double t748;
  double t759;
  double t765;
  double t770;
  double t947;
  double t960;
  double t961;
  double t976;
  double t978;
  double t987;
  double t989;
  double t992;
  double t997;
  double t1005;
  double t1008;
  double t1062;
  double t1074;
  double t1090;
  double t1098;
  double t1100;
  double t1115;
  double t1122;
  double t1124;
  double t1140;
  double t1142;
  double t1255;
  double t1267;
  double t1272;
  double t1365;
  double t1367;
  double t1349;
  double t1354;
  double t1371;
  double t1375;
  double t1376;
  double t1377;
  double t1386;
  double t1387;
  double t1391;
  double t1399;
  double t1333;
  double t1340;
  double t1356;
  double t1359;
  double t1382;
  double t1406;
  double t1408;
  double t1435;
  double t1443;
  double t1449;
  double t1460;
  double t1471;
  double t1316;
  double t1324;
  double t1343;
  double t1348;
  double t1415;
  double t1473;
  double t1474;
  double t1479;
  double t1482;
  double t1483;
  double t1489;
  double t1490;
  double t1313;
  double t1314;
  double t1506;
  double t1508;
  double t1513;
  double t1516;
  double t1517;
  double t1330;
  double t1331;
  double t1477;
  double t1494;
  double t1497;
  double t1284;
  double t1288;
  double t1315;
  double t1498;
  double t1505;
  double t1523;
  double t1524;
  double t1526;
  double t1527;
  double t1532;
  double t1534;
  double t1536;
  double t1538;
  double t1541;
  double t1551;
  double t1558;
  double t1560;
  double t1561;
  double t1563;
  double t1275;
  double t1276;
  double t1282;
  double t1295;
  double t1304;
  double t1525;
  double t1569;
  double t1580;
  double t1593;
  double t1595;
  double t1597;
  double t1599;
  double t1601;
  double t1609;
  double t1611;
  double t1616;
  double t1620;
  double t1633;
  double t1645;
  double t1647;
  double t1582;
  double t1602;
  double t1604;
  double t1629;
  double t1649;
  double t1651;
  double t1659;
  double t1660;
  double t1667;
  double t1243;
  double t1247;
  double t1251;
  double t1606;
  double t1694;
  double t1721;
  double t1743;
  double t1782;
  double t1795;
  double t1815;
  double t1829;
  double t1837;
  double t1848;
  double t1864;
  double t1872;
  double t2030;
  double t2041;
  t217 = Cos(var1[11]);
  t188 = Cos(var1[12]);
  t194 = Sin(var1[11]);
  t225 = Sin(var1[12]);
  t271 = Cos(var1[10]);
  t204 = t188*t194;
  t229 = t217*t225;
  t248 = 0. + t204 + t229;
  t163 = Sin(var1[10]);
  t277 = -1.*t217*t188;
  t292 = t194*t225;
  t312 = 0. + t277 + t292;
  t369 = Sin(var1[9]);
  t263 = t163*t248;
  t315 = t271*t312;
  t317 = 0. + t263 + t315;
  t127 = Cos(var1[9]);
  t383 = t271*t248;
  t390 = -1.*t163*t312;
  t399 = 0. + t383 + t390;
  t108 = Cos(var1[8]);
  t459 = -1.*t369*t317;
  t485 = t127*t399;
  t504 = 0. + t459 + t485;
  t355 = t127*t317;
  t443 = t369*t399;
  t446 = 0. + t355 + t443;
  t516 = Sin(var1[8]);
  t547 = Cos(var1[6]);
  t451 = t108*t446;
  t520 = t504*t516;
  t529 = 0. + t451 + t520;
  t89 = Sin(var1[6]);
  t567 = Sin(var1[7]);
  t569 = t108*t504;
  t575 = -1.*t446*t516;
  t586 = 0. + t569 + t575;
  t606 = t567*t586;
  t619 = 0. + t606;
  t665 = Sin(var1[5]);
  t541 = -1.*t89*t529;
  t633 = t547*t619;
  t639 = 0. + t541 + t633;
  t88 = Cos(var1[5]);
  t670 = t547*t529;
  t673 = t89*t619;
  t674 = 0. + t670 + t673;
  t32 = Sin(var1[3]);
  t792 = t217*t188;
  t797 = -1.*t194*t225;
  t799 = 0. + t792 + t797;
  t783 = -1.*t163*t248;
  t802 = t271*t799;
  t804 = 0. + t783 + t802;
  t816 = t163*t799;
  t818 = 0. + t383 + t816;
  t831 = t127*t804;
  t840 = -1.*t369*t818;
  t854 = 0. + t831 + t840;
  t810 = t369*t804;
  t819 = t127*t818;
  t823 = 0. + t810 + t819;
  t829 = t108*t823;
  t861 = t854*t516;
  t878 = 0. + t829 + t861;
  t898 = t108*t854;
  t904 = -1.*t823*t516;
  t905 = 0. + t898 + t904;
  t908 = t567*t905;
  t912 = 0. + t908;
  t705 = Cos(var1[3]);
  t708 = Cos(var1[4]);
  t711 = Cos(var1[7]);
  t732 = Sin(var1[4]);
  t883 = -1.*t89*t878;
  t916 = t547*t912;
  t939 = 0. + t883 + t916;
  t954 = t547*t878;
  t957 = t89*t912;
  t958 = 0. + t954 + t957;
  t1024 = -1.*t711;
  t1027 = 0. + t1024;
  t1040 = t547*t1027;
  t1061 = 0. + t1040;
  t1064 = t1027*t89;
  t1065 = 0. + t1064;
  t643 = t88*t639;
  t683 = -1.*t665*t674;
  t690 = 0. + t643 + t683;
  t714 = t711*t586;
  t719 = 0. + t714;
  t724 = t708*t719;
  t737 = t665*t639;
  t748 = t88*t674;
  t759 = 0. + t737 + t748;
  t765 = t732*t759;
  t770 = 0. + t724 + t765;
  t947 = t88*t939;
  t960 = -1.*t665*t958;
  t961 = 0. + t947 + t960;
  t976 = t711*t905;
  t978 = 0. + t976;
  t987 = t708*t978;
  t989 = t665*t939;
  t992 = t88*t958;
  t997 = 0. + t989 + t992;
  t1005 = t732*t997;
  t1008 = 0. + t987 + t1005;
  t1062 = t88*t1061;
  t1074 = -1.*t665*t1065;
  t1090 = 0. + t1062 + t1074;
  t1098 = t1061*t665;
  t1100 = t88*t1065;
  t1115 = 0. + t1098 + t1100;
  t1122 = t732*t1115;
  t1124 = 0. + t567;
  t1140 = t708*t1124;
  t1142 = 0. + t1122 + t1140;
  t1255 = -1.*t732*t978;
  t1267 = t708*t997;
  t1272 = 0. + t1255 + t1267;
  t1365 = -1.*t188;
  t1367 = 1. + t1365;
  t1349 = -1.*t217;
  t1354 = 1. + t1349;
  t1371 = -0.05315*t1367;
  t1375 = -0.02565*t188;
  t1376 = 0.0047000000000001485*t225;
  t1377 = 0. + t1371 + t1375 + t1376;
  t1386 = -1.03354*t1367;
  t1387 = -1.03824*t188;
  t1391 = 0.027500000000000004*t225;
  t1399 = 0. + t1386 + t1387 + t1391;
  t1333 = -1.*t271;
  t1340 = 1. + t1333;
  t1356 = -0.62554*t1354;
  t1359 = 0.01315*t194;
  t1382 = t194*t1377;
  t1406 = t217*t1399;
  t1408 = 0. + t1356 + t1359 + t1382 + t1406;
  t1435 = -0.01315*t1354;
  t1443 = -0.62554*t194;
  t1449 = t217*t1377;
  t1460 = -1.*t194*t1399;
  t1471 = 0. + t1435 + t1443 + t1449 + t1460;
  t1316 = -1.*t127;
  t1324 = 1. + t1316;
  t1343 = -0.03315*t1340;
  t1348 = -0.19074*t163;
  t1415 = -1.*t163*t1408;
  t1473 = t271*t1471;
  t1474 = 0. + t1343 + t1348 + t1415 + t1473;
  t1479 = -0.19074*t1340;
  t1482 = 0.03315*t163;
  t1483 = t271*t1408;
  t1489 = t163*t1471;
  t1490 = 0. + t1479 + t1482 + t1483 + t1489;
  t1313 = -1.*t108;
  t1314 = 1. + t1313;
  t1506 = -0.08055*t1324;
  t1508 = -0.13004*t369;
  t1513 = t127*t1474;
  t1516 = -1.*t369*t1490;
  t1517 = 0. + t1506 + t1508 + t1513 + t1516;
  t1330 = -0.13004*t1324;
  t1331 = 0.08055*t369;
  t1477 = t369*t1474;
  t1494 = t127*t1490;
  t1497 = 0. + t1330 + t1331 + t1477 + t1494;
  t1284 = -1.*t547;
  t1288 = 1. + t1284;
  t1315 = -0.01004*t1314;
  t1498 = t108*t1497;
  t1505 = 0.08055*t516;
  t1523 = t1517*t516;
  t1524 = 0. + t1315 + t1498 + t1505 + t1523;
  t1526 = -1.*t711;
  t1527 = 1. + t1526;
  t1532 = 0.135*t1527;
  t1534 = 0.1306*t711;
  t1536 = 0.08055*t567;
  t1538 = -0.08055*t1314;
  t1541 = t108*t1517;
  t1551 = -0.01004*t516;
  t1558 = -1.*t1497*t516;
  t1560 = 0. + t1538 + t1541 + t1551 + t1558;
  t1561 = t567*t1560;
  t1563 = 0. + t1532 + t1534 + t1536 + t1561;
  t1275 = t708*t1115;
  t1276 = -1.*t732*t1124;
  t1282 = 0. + t1275 + t1276;
  t1295 = 0.135*t1288;
  t1304 = 0.07996*t89;
  t1525 = -1.*t89*t1524;
  t1569 = t547*t1563;
  t1580 = 0. + t1295 + t1304 + t1525 + t1569;
  t1593 = 0.07996*t1288;
  t1595 = -0.135*t89;
  t1597 = t547*t1524;
  t1599 = t89*t1563;
  t1601 = 0. + t1593 + t1595 + t1597 + t1599;
  t1609 = -0.08055*t1527;
  t1611 = 0.004400000000000015*t567;
  t1616 = t711*t1560;
  t1620 = 0. + t1609 + t1611 + t1616;
  t1633 = t665*t1580;
  t1645 = t88*t1601;
  t1647 = 0. + t1633 + t1645;
  t1582 = t88*t1580;
  t1602 = -1.*t665*t1601;
  t1604 = 0. + t1582 + t1602;
  t1629 = -1.*t732*t1620;
  t1649 = t708*t1647;
  t1651 = 0. + t1629 + t1649;
  t1659 = t708*t1620;
  t1660 = t732*t1647;
  t1667 = 0. + t1659 + t1660;
  t1243 = -1.*t732*t719;
  t1247 = t708*t759;
  t1251 = 0. + t1243 + t1247;
  t1606 = t1090*t1604;
  t1694 = -1.*t1604*t961;
  t1721 = -1.*t1090*t1604;
  t1743 = t1604*t690;
  t1782 = t1604*t961;
  t1795 = -1.*t1604*t690;
  t1815 = t1124*t1620;
  t1829 = -1.*t1620*t978;
  t1837 = -1.*t1124*t1620;
  t1848 = t1620*t719;
  t1864 = t1620*t978;
  t1872 = -1.*t1620*t719;
  t2030 = t1524*t878;
  t2041 = -1.*t1524*t529;
  p_output1[0]=0. - 1.*t32*t690 + t705*t770;
  p_output1[1]=0. + t1008*t705 - 1.*t32*t961;
  p_output1[2]=0. - 1.*t1090*t32 + t1142*t705;
  p_output1[3]=0;
  p_output1[4]=0;
  p_output1[5]=0;
  p_output1[6]=0. + t690*t705 + t32*t770;
  p_output1[7]=0. + t1008*t32 + t705*t961;
  p_output1[8]=0. + t1142*t32 + t1090*t705;
  p_output1[9]=0;
  p_output1[10]=0;
  p_output1[11]=0;
  p_output1[12]=t1251;
  p_output1[13]=t1272;
  p_output1[14]=t1282;
  p_output1[15]=0;
  p_output1[16]=0;
  p_output1[17]=0;
  p_output1[18]=t1272*(t1606 + t1282*t1651 + t1142*t1667) + t1282*(-1.*t1272*t1651 - 1.*t1008*t1667 + t1694);
  p_output1[19]=t1251*(-1.*t1282*t1651 - 1.*t1142*t1667 + t1721) + t1282*(t1251*t1651 + t1743 + t1667*t770);
  p_output1[20]=t1251*(t1272*t1651 + t1008*t1667 + t1782) + t1272*(-1.*t1251*t1651 + t1795 - 1.*t1667*t770);
  p_output1[21]=t1251;
  p_output1[22]=t1272;
  p_output1[23]=t1282;
  p_output1[24]=(t1606 + t1115*t1647 + t1815)*t961 + t1090*(t1694 + t1829 - 1.*t1647*t997);
  p_output1[25]=(-1.*t1115*t1647 + t1721 + t1837)*t690 + t1090*(t1743 + t1848 + t1647*t759);
  p_output1[26]=(t1795 + t1872 - 1.*t1647*t759)*t961 + t690*(t1782 + t1864 + t1647*t997);
  p_output1[27]=t690;
  p_output1[28]=t961;
  p_output1[29]=t1090;
  p_output1[30]=t1124*(t1829 - 1.*t1580*t939 - 1.*t1601*t958) + (t1061*t1580 + t1065*t1601 + t1815)*t978;
  p_output1[31]=t1124*(t1848 + t1580*t639 + t1601*t674) + (-1.*t1061*t1580 - 1.*t1065*t1601 + t1837)*t719;
  p_output1[32]=t719*(t1864 + t1580*t939 + t1601*t958) + (t1872 - 1.*t1580*t639 - 1.*t1601*t674)*t978;
  p_output1[33]=t719;
  p_output1[34]=t978;
  p_output1[35]=t1124;
  p_output1[36]=-0.135*t529 + 0.07996*t619 + t1124*(t1829 - 1.*t1524*t878 - 1.*t1563*t912) + (0. + t1027*t1563 + t1815)*t978;
  p_output1[37]=t1124*(t1848 + t1524*t529 + t1563*t619) + (0. - 1.*t1027*t1563 + t1837)*t719 - 0.135*t878 + 0.07996*t912;
  p_output1[38]=0. + 0.07996*t1027 + t719*(t1864 + t2030 + t1563*t912) + (t1872 + t2041 - 1.*t1563*t619)*t978;
  p_output1[39]=t719;
  p_output1[40]=t978;
  p_output1[41]=t1124;
  p_output1[42]=0. + 0.135*t586 - 0.1306*t878;
  p_output1[43]=0. + 0.1306*t529 + 0.135*t905;
  p_output1[44]=-0.08055 + (0. + t2041 - 1.*t1560*t586)*t878 + t529*(0. + t2030 + t1560*t905);
  p_output1[45]=t529;
  p_output1[46]=t878;
  p_output1[47]=0.;
  p_output1[48]=0. + 0.08055*t446 - 0.01004*t504 - 1.*t1497*t823 - 1.*t1517*t854;
  p_output1[49]=0. + t1497*t446 + t1517*t504 + 0.08055*t823 - 0.01004*t854;
  p_output1[50]=0.;
  p_output1[51]=0.;
  p_output1[52]=0.;
  p_output1[53]=1.;
  p_output1[54]=0. + 0.08055*t317 - 0.13004*t399 - 1.*t1474*t804 - 1.*t1490*t818;
  p_output1[55]=0. + t1490*t317 + t1474*t399 - 0.13004*t804 + 0.08055*t818;
  p_output1[56]=0.;
  p_output1[57]=0.;
  p_output1[58]=0.;
  p_output1[59]=1.;
  p_output1[60]=0. - 0.19074*t248 - 1.*t1408*t248 + 0.03315*t312 - 1.*t1471*t799;
  p_output1[61]=0. + 0.03315*t248 + t1471*t248 + t1408*t312 - 0.19074*t799;
  p_output1[62]=0.;
  p_output1[63]=0.;
  p_output1[64]=0.;
  p_output1[65]=1.;
  p_output1[66]=0. - 0.01315*t188 - 1.*t1377*t188 - 0.62554*t225 - 1.*t1399*t225;
  p_output1[67]=0. - 0.62554*t188 - 1.*t1399*t188 + 0.01315*t225 + t1377*t225;
  p_output1[68]=0.;
  p_output1[69]=0.;
  p_output1[70]=0.;
  p_output1[71]=1.;
  p_output1[72]=-0.027500000000000004;
  p_output1[73]=0.0047000000000001485;
  p_output1[74]=0.;
  p_output1[75]=0.;
  p_output1[76]=0.;
  p_output1[77]=1.;
  p_output1[78]=0;
  p_output1[79]=0;
  p_output1[80]=0;
  p_output1[81]=0;
  p_output1[82]=0;
  p_output1[83]=0;
  p_output1[84]=0;
  p_output1[85]=0;
  p_output1[86]=0;
  p_output1[87]=0;
  p_output1[88]=0;
  p_output1[89]=0;
  p_output1[90]=0;
  p_output1[91]=0;
  p_output1[92]=0;
  p_output1[93]=0;
  p_output1[94]=0;
  p_output1[95]=0;
  p_output1[96]=0;
  p_output1[97]=0;
  p_output1[98]=0;
  p_output1[99]=0;
  p_output1[100]=0;
  p_output1[101]=0;
  p_output1[102]=0;
  p_output1[103]=0;
  p_output1[104]=0;
  p_output1[105]=0;
  p_output1[106]=0;
  p_output1[107]=0;
  p_output1[108]=0;
  p_output1[109]=0;
  p_output1[110]=0;
  p_output1[111]=0;
  p_output1[112]=0;
  p_output1[113]=0;
  p_output1[114]=0;
  p_output1[115]=0;
  p_output1[116]=0;
  p_output1[117]=0;
  p_output1[118]=0;
  p_output1[119]=0;
}



void Jb_LeftToe_src(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}

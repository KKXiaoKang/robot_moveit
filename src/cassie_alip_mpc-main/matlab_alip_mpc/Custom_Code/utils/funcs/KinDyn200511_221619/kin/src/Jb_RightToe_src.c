/*
 * Automatically Generated from Mathematica.
 * Mon 11 May 2020 22:30:48 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jb_RightToe_src.h"

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
  double t252;
  double t164;
  double t172;
  double t272;
  double t292;
  double t216;
  double t275;
  double t283;
  double t158;
  double t300;
  double t326;
  double t327;
  double t354;
  double t290;
  double t338;
  double t340;
  double t143;
  double t367;
  double t378;
  double t381;
  double t428;
  double t344;
  double t416;
  double t419;
  double t125;
  double t443;
  double t447;
  double t457;
  double t476;
  double t422;
  double t459;
  double t464;
  double t73;
  double t484;
  double t487;
  double t507;
  double t510;
  double t513;
  double t517;
  double t50;
  double t561;
  double t569;
  double t573;
  double t472;
  double t527;
  double t536;
  double t578;
  double t25;
  double t735;
  double t740;
  double t744;
  double t733;
  double t758;
  double t764;
  double t773;
  double t778;
  double t767;
  double t782;
  double t786;
  double t795;
  double t800;
  double t804;
  double t792;
  double t807;
  double t812;
  double t844;
  double t868;
  double t870;
  double t879;
  double t880;
  double t614;
  double t633;
  double t634;
  double t662;
  double t908;
  double t910;
  double t934;
  double t828;
  double t885;
  double t891;
  double t1027;
  double t1034;
  double t1049;
  double t1050;
  double t1039;
  double t1043;
  double t553;
  double t593;
  double t601;
  double t644;
  double t648;
  double t652;
  double t673;
  double t682;
  double t684;
  double t690;
  double t696;
  double t892;
  double t938;
  double t946;
  double t958;
  double t981;
  double t989;
  double t993;
  double t996;
  double t1001;
  double t1003;
  double t1007;
  double t1044;
  double t1054;
  double t1061;
  double t1077;
  double t1081;
  double t1087;
  double t1094;
  double t1101;
  double t1103;
  double t1109;
  double t1228;
  double t1233;
  double t1237;
  double t1340;
  double t1345;
  double t1328;
  double t1331;
  double t1349;
  double t1350;
  double t1351;
  double t1353;
  double t1360;
  double t1364;
  double t1365;
  double t1371;
  double t1320;
  double t1322;
  double t1334;
  double t1339;
  double t1359;
  double t1377;
  double t1385;
  double t1389;
  double t1394;
  double t1397;
  double t1403;
  double t1423;
  double t1301;
  double t1313;
  double t1323;
  double t1324;
  double t1386;
  double t1424;
  double t1425;
  double t1435;
  double t1442;
  double t1444;
  double t1450;
  double t1453;
  double t1286;
  double t1288;
  double t1314;
  double t1315;
  double t1429;
  double t1454;
  double t1456;
  double t1458;
  double t1461;
  double t1469;
  double t1472;
  double t1474;
  double t1253;
  double t1257;
  double t1269;
  double t1270;
  double t1271;
  double t1275;
  double t1276;
  double t1289;
  double t1297;
  double t1457;
  double t1479;
  double t1483;
  double t1484;
  double t1486;
  double t1488;
  double t1491;
  double t1493;
  double t1495;
  double t1496;
  double t1242;
  double t1248;
  double t1249;
  double t1511;
  double t1512;
  double t1515;
  double t1517;
  double t1520;
  double t1259;
  double t1267;
  double t1487;
  double t1500;
  double t1504;
  double t1525;
  double t1527;
  double t1528;
  double t1529;
  double t1535;
  double t1537;
  double t1539;
  double t1509;
  double t1521;
  double t1522;
  double t1534;
  double t1545;
  double t1546;
  double t1552;
  double t1555;
  double t1559;
  double t1211;
  double t1218;
  double t1220;
  double t1523;
  double t1584;
  double t1617;
  double t1632;
  double t1650;
  double t1671;
  double t1710;
  double t1723;
  double t1733;
  double t1747;
  double t1758;
  double t1770;
  double t1946;
  double t1968;
  t252 = Cos(var1[18]);
  t164 = Cos(var1[19]);
  t172 = Sin(var1[18]);
  t272 = Sin(var1[19]);
  t292 = Cos(var1[17]);
  t216 = t164*t172;
  t275 = t252*t272;
  t283 = 0. + t216 + t275;
  t158 = Sin(var1[17]);
  t300 = -1.*t252*t164;
  t326 = t172*t272;
  t327 = 0. + t300 + t326;
  t354 = Cos(var1[16]);
  t290 = t158*t283;
  t338 = t292*t327;
  t340 = 0. + t290 + t338;
  t143 = Sin(var1[16]);
  t367 = t292*t283;
  t378 = -1.*t158*t327;
  t381 = 0. + t367 + t378;
  t428 = Cos(var1[15]);
  t344 = -1.*t143*t340;
  t416 = t354*t381;
  t419 = 0. + t344 + t416;
  t125 = Sin(var1[15]);
  t443 = t354*t340;
  t447 = t143*t381;
  t457 = 0. + t443 + t447;
  t476 = Cos(var1[13]);
  t422 = t125*t419;
  t459 = t428*t457;
  t464 = 0. + t422 + t459;
  t73 = Sin(var1[13]);
  t484 = Sin(var1[14]);
  t487 = t428*t419;
  t507 = -1.*t125*t457;
  t510 = 0. + t487 + t507;
  t513 = t484*t510;
  t517 = 0. + t513;
  t50 = Cos(var1[5]);
  t561 = t476*t464;
  t569 = t73*t517;
  t573 = 0. + t561 + t569;
  t472 = -1.*t73*t464;
  t527 = t476*t517;
  t536 = 0. + t472 + t527;
  t578 = Sin(var1[5]);
  t25 = Sin(var1[3]);
  t735 = t252*t164;
  t740 = -1.*t172*t272;
  t744 = 0. + t735 + t740;
  t733 = -1.*t158*t283;
  t758 = t292*t744;
  t764 = 0. + t733 + t758;
  t773 = t158*t744;
  t778 = 0. + t367 + t773;
  t767 = t143*t764;
  t782 = t354*t778;
  t786 = 0. + t767 + t782;
  t795 = t354*t764;
  t800 = -1.*t143*t778;
  t804 = 0. + t795 + t800;
  t792 = t428*t786;
  t807 = t125*t804;
  t812 = 0. + t792 + t807;
  t844 = -1.*t125*t786;
  t868 = t428*t804;
  t870 = 0. + t844 + t868;
  t879 = t484*t870;
  t880 = 0. + t879;
  t614 = Cos(var1[3]);
  t633 = Cos(var1[4]);
  t634 = Cos(var1[14]);
  t662 = Sin(var1[4]);
  t908 = t476*t812;
  t910 = t73*t880;
  t934 = 0. + t908 + t910;
  t828 = -1.*t73*t812;
  t885 = t476*t880;
  t891 = 0. + t828 + t885;
  t1027 = -1.*t634;
  t1034 = 0. + t1027;
  t1049 = t1034*t73;
  t1050 = 0. + t1049;
  t1039 = t476*t1034;
  t1043 = 0. + t1039;
  t553 = t50*t536;
  t593 = -1.*t573*t578;
  t601 = 0. + t553 + t593;
  t644 = t634*t510;
  t648 = 0. + t644;
  t652 = t633*t648;
  t673 = t50*t573;
  t682 = t536*t578;
  t684 = 0. + t673 + t682;
  t690 = t662*t684;
  t696 = 0. + t652 + t690;
  t892 = t50*t891;
  t938 = -1.*t934*t578;
  t946 = 0. + t892 + t938;
  t958 = t634*t870;
  t981 = 0. + t958;
  t989 = t633*t981;
  t993 = t50*t934;
  t996 = t891*t578;
  t1001 = 0. + t993 + t996;
  t1003 = t662*t1001;
  t1007 = 0. + t989 + t1003;
  t1044 = t1043*t50;
  t1054 = -1.*t1050*t578;
  t1061 = 0. + t1044 + t1054;
  t1077 = 0. + t484;
  t1081 = t633*t1077;
  t1087 = t50*t1050;
  t1094 = t1043*t578;
  t1101 = 0. + t1087 + t1094;
  t1103 = t662*t1101;
  t1109 = 0. + t1081 + t1103;
  t1228 = -1.*t981*t662;
  t1233 = t633*t1001;
  t1237 = 0. + t1228 + t1233;
  t1340 = -1.*t164;
  t1345 = 1. + t1340;
  t1328 = -1.*t252;
  t1331 = 1. + t1328;
  t1349 = -0.05315*t1345;
  t1350 = -0.02565*t164;
  t1351 = 0.0047000000000001485*t272;
  t1353 = 0. + t1349 + t1350 + t1351;
  t1360 = -1.03354*t1345;
  t1364 = -1.03824*t164;
  t1365 = 0.027500000000000004*t272;
  t1371 = 0. + t1360 + t1364 + t1365;
  t1320 = -1.*t292;
  t1322 = 1. + t1320;
  t1334 = -0.62554*t1331;
  t1339 = 0.01315*t172;
  t1359 = t172*t1353;
  t1377 = t252*t1371;
  t1385 = 0. + t1334 + t1339 + t1359 + t1377;
  t1389 = -0.01315*t1331;
  t1394 = -0.62554*t172;
  t1397 = t252*t1353;
  t1403 = -1.*t172*t1371;
  t1423 = 0. + t1389 + t1394 + t1397 + t1403;
  t1301 = -1.*t354;
  t1313 = 1. + t1301;
  t1323 = -0.03315*t1322;
  t1324 = -0.19074*t158;
  t1386 = -1.*t158*t1385;
  t1424 = t292*t1423;
  t1425 = 0. + t1323 + t1324 + t1386 + t1424;
  t1435 = -0.19074*t1322;
  t1442 = 0.03315*t158;
  t1444 = t292*t1385;
  t1450 = t158*t1423;
  t1453 = 0. + t1435 + t1442 + t1444 + t1450;
  t1286 = -1.*t428;
  t1288 = 1. + t1286;
  t1314 = -0.13004*t1313;
  t1315 = 0.08055*t143;
  t1429 = t143*t1425;
  t1454 = t354*t1453;
  t1456 = 0. + t1314 + t1315 + t1429 + t1454;
  t1458 = -0.08055*t1313;
  t1461 = -0.13004*t143;
  t1469 = t354*t1425;
  t1472 = -1.*t143*t1453;
  t1474 = 0. + t1458 + t1461 + t1469 + t1472;
  t1253 = -1.*t476;
  t1257 = 1. + t1253;
  t1269 = -1.*t634;
  t1270 = 1. + t1269;
  t1271 = -0.135*t1270;
  t1275 = -0.1306*t634;
  t1276 = 0.08055*t484;
  t1289 = -0.08055*t1288;
  t1297 = -0.01004*t125;
  t1457 = -1.*t125*t1456;
  t1479 = t428*t1474;
  t1483 = 0. + t1289 + t1297 + t1457 + t1479;
  t1484 = t484*t1483;
  t1486 = 0. + t1271 + t1275 + t1276 + t1484;
  t1488 = -0.01004*t1288;
  t1491 = 0.08055*t125;
  t1493 = t428*t1456;
  t1495 = t125*t1474;
  t1496 = 0. + t1488 + t1491 + t1493 + t1495;
  t1242 = -1.*t1077*t662;
  t1248 = t633*t1101;
  t1249 = 0. + t1242 + t1248;
  t1511 = 0.07996*t1257;
  t1512 = 0.135*t73;
  t1515 = t73*t1486;
  t1517 = t476*t1496;
  t1520 = 0. + t1511 + t1512 + t1515 + t1517;
  t1259 = -0.135*t1257;
  t1267 = 0.07996*t73;
  t1487 = t476*t1486;
  t1500 = -1.*t73*t1496;
  t1504 = 0. + t1259 + t1267 + t1487 + t1500;
  t1525 = -0.08055*t1270;
  t1527 = -0.004400000000000015*t484;
  t1528 = t634*t1483;
  t1529 = 0. + t1525 + t1527 + t1528;
  t1535 = t50*t1520;
  t1537 = t1504*t578;
  t1539 = 0. + t1535 + t1537;
  t1509 = t50*t1504;
  t1521 = -1.*t1520*t578;
  t1522 = 0. + t1509 + t1521;
  t1534 = -1.*t1529*t662;
  t1545 = t633*t1539;
  t1546 = 0. + t1534 + t1545;
  t1552 = t633*t1529;
  t1555 = t662*t1539;
  t1559 = 0. + t1552 + t1555;
  t1211 = -1.*t648*t662;
  t1218 = t633*t684;
  t1220 = 0. + t1211 + t1218;
  t1523 = t1061*t1522;
  t1584 = -1.*t1522*t946;
  t1617 = -1.*t1061*t1522;
  t1632 = t1522*t601;
  t1650 = t1522*t946;
  t1671 = -1.*t1522*t601;
  t1710 = t1077*t1529;
  t1723 = -1.*t1529*t981;
  t1733 = -1.*t1077*t1529;
  t1747 = t1529*t648;
  t1758 = t1529*t981;
  t1770 = -1.*t1529*t648;
  t1946 = t1496*t812;
  t1968 = -1.*t1496*t464;
  p_output1[0]=0. - 1.*t25*t601 + t614*t696;
  p_output1[1]=0. + t1007*t614 - 1.*t25*t946;
  p_output1[2]=0. - 1.*t1061*t25 + t1109*t614;
  p_output1[3]=0;
  p_output1[4]=0;
  p_output1[5]=0;
  p_output1[6]=0. + t601*t614 + t25*t696;
  p_output1[7]=0. + t1007*t25 + t614*t946;
  p_output1[8]=0. + t1109*t25 + t1061*t614;
  p_output1[9]=0;
  p_output1[10]=0;
  p_output1[11]=0;
  p_output1[12]=t1220;
  p_output1[13]=t1237;
  p_output1[14]=t1249;
  p_output1[15]=0;
  p_output1[16]=0;
  p_output1[17]=0;
  p_output1[18]=t1237*(t1523 + t1249*t1546 + t1109*t1559) + t1249*(-1.*t1237*t1546 - 1.*t1007*t1559 + t1584);
  p_output1[19]=t1220*(-1.*t1249*t1546 - 1.*t1109*t1559 + t1617) + t1249*(t1220*t1546 + t1632 + t1559*t696);
  p_output1[20]=t1220*(t1237*t1546 + t1007*t1559 + t1650) + t1237*(-1.*t1220*t1546 + t1671 - 1.*t1559*t696);
  p_output1[21]=t1220;
  p_output1[22]=t1237;
  p_output1[23]=t1249;
  p_output1[24]=t1061*(-1.*t1001*t1539 + t1584 + t1723) + (t1523 + t1101*t1539 + t1710)*t946;
  p_output1[25]=(-1.*t1101*t1539 + t1617 + t1733)*t601 + t1061*(t1632 + t1747 + t1539*t684);
  p_output1[26]=(t1001*t1539 + t1650 + t1758)*t601 + (t1671 + t1770 - 1.*t1539*t684)*t946;
  p_output1[27]=t601;
  p_output1[28]=t946;
  p_output1[29]=t1061;
  p_output1[30]=t1077*(t1723 - 1.*t1504*t891 - 1.*t1520*t934) + (t1043*t1504 + t1050*t1520 + t1710)*t981;
  p_output1[31]=t1077*(t1747 + t1504*t536 + t1520*t573) + (-1.*t1043*t1504 - 1.*t1050*t1520 + t1733)*t648;
  p_output1[32]=t648*(t1758 + t1504*t891 + t1520*t934) + (t1770 - 1.*t1504*t536 - 1.*t1520*t573)*t981;
  p_output1[33]=t648;
  p_output1[34]=t981;
  p_output1[35]=t1077;
  p_output1[36]=0;
  p_output1[37]=0;
  p_output1[38]=0;
  p_output1[39]=0;
  p_output1[40]=0;
  p_output1[41]=0;
  p_output1[42]=0;
  p_output1[43]=0;
  p_output1[44]=0;
  p_output1[45]=0;
  p_output1[46]=0;
  p_output1[47]=0;
  p_output1[48]=0;
  p_output1[49]=0;
  p_output1[50]=0;
  p_output1[51]=0;
  p_output1[52]=0;
  p_output1[53]=0;
  p_output1[54]=0;
  p_output1[55]=0;
  p_output1[56]=0;
  p_output1[57]=0;
  p_output1[58]=0;
  p_output1[59]=0;
  p_output1[60]=0;
  p_output1[61]=0;
  p_output1[62]=0;
  p_output1[63]=0;
  p_output1[64]=0;
  p_output1[65]=0;
  p_output1[66]=0;
  p_output1[67]=0;
  p_output1[68]=0;
  p_output1[69]=0;
  p_output1[70]=0;
  p_output1[71]=0;
  p_output1[72]=0;
  p_output1[73]=0;
  p_output1[74]=0;
  p_output1[75]=0;
  p_output1[76]=0;
  p_output1[77]=0;
  p_output1[78]=0.135*t464 + 0.07996*t517 + t1077*(t1723 - 1.*t1496*t812 - 1.*t1486*t880) + (0. + t1034*t1486 + t1710)*t981;
  p_output1[79]=t1077*(t1747 + t1496*t464 + t1486*t517) + (0. - 1.*t1034*t1486 + t1733)*t648 + 0.135*t812 + 0.07996*t880;
  p_output1[80]=0. + 0.07996*t1034 + t648*(t1758 + t1946 + t1486*t880) + (t1770 + t1968 - 1.*t1486*t517)*t981;
  p_output1[81]=t648;
  p_output1[82]=t981;
  p_output1[83]=t1077;
  p_output1[84]=0. - 0.135*t510 + 0.1306*t812;
  p_output1[85]=0. - 0.1306*t464 - 0.135*t870;
  p_output1[86]=-0.08055 + (0. + t1968 - 1.*t1483*t510)*t812 + t464*(0. + t1946 + t1483*t870);
  p_output1[87]=t464;
  p_output1[88]=t812;
  p_output1[89]=0.;
  p_output1[90]=0. - 0.01004*t419 + 0.08055*t457 - 1.*t1456*t786 - 1.*t1474*t804;
  p_output1[91]=0. + t1474*t419 + t1456*t457 + 0.08055*t786 - 0.01004*t804;
  p_output1[92]=0.;
  p_output1[93]=0.;
  p_output1[94]=0.;
  p_output1[95]=1.;
  p_output1[96]=0. + 0.08055*t340 - 0.13004*t381 - 1.*t1425*t764 - 1.*t1453*t778;
  p_output1[97]=0. + t1453*t340 + t1425*t381 - 0.13004*t764 + 0.08055*t778;
  p_output1[98]=0.;
  p_output1[99]=0.;
  p_output1[100]=0.;
  p_output1[101]=1.;
  p_output1[102]=0. - 0.19074*t283 - 1.*t1385*t283 + 0.03315*t327 - 1.*t1423*t744;
  p_output1[103]=0. + 0.03315*t283 + t1423*t283 + t1385*t327 - 0.19074*t744;
  p_output1[104]=0.;
  p_output1[105]=0.;
  p_output1[106]=0.;
  p_output1[107]=1.;
  p_output1[108]=0. - 0.01315*t164 - 1.*t1353*t164 - 0.62554*t272 - 1.*t1371*t272;
  p_output1[109]=0. - 0.62554*t164 - 1.*t1371*t164 + 0.01315*t272 + t1353*t272;
  p_output1[110]=0.;
  p_output1[111]=0.;
  p_output1[112]=0.;
  p_output1[113]=1.;
  p_output1[114]=-0.027500000000000004;
  p_output1[115]=0.0047000000000001485;
  p_output1[116]=0.;
  p_output1[117]=0.;
  p_output1[118]=0.;
  p_output1[119]=1.;
}



void Jb_RightToe_src(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}

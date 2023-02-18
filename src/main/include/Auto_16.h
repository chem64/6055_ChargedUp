#pragma once

//Auto  -  drive for 16 feet
//feetL,velL  - velocity is feet/sec
const int auto16_size = 362; 
const double auto16_L[][2] = {
        {0,0},
        {0.0004,0.0384},
        {0.0012,0.0768},
        {0.0025,0.128},
        {0.0042,0.1776},
        {0.0064,0.2262},
        {0.0092,0.2759},
        {0.0125,0.326},
        {0.0161,0.3756},
        {0.0205,0.4255},
        {0.0252,0.4758},
        {0.0304,0.5256},
        {0.0362,0.5757},
        {0.0424,0.6255},
        {0.0492,0.6755},
        {0.0565,0.7258},
        {0.0643,0.7757},
        {0.0724,0.8255},
        {0.0813,0.8755},
        {0.0905,0.9256},
        {0.1003,0.9756},
        {0.1104,1.0254},
        {0.1211,1.0751},
        {0.1325,1.1253},
        {0.1442,1.1753},
        {0.1565,1.2253},
        {0.1692,1.2754},
        {0.1825,1.3254},
        {0.1962,1.3753},
        {0.2104,1.4251},
        {0.2252,1.4751},
        {0.2405,1.5253},
        {0.2562,1.5754},
        {0.2724,1.6252},
        {0.2892,1.6752},
        {0.3065,1.7253},
        {0.3242,1.7753},
        {0.3424,1.8252},
        {0.3612,1.8752},
        {0.3805,1.9253},
        {0.4003,1.9753},
        {0.4205,2.0253},
        {0.4411,2.0751},
        {0.4625,2.1252},
        {0.4842,2.1752},
        {0.5064,2.225},
        {0.5292,2.2751},
        {0.5524,2.3251},
        {0.5761,2.3751},
        {0.6004,2.425},
        {0.6253,2.4752},
        {0.6504,2.5252},
        {0.6762,2.5751},
        {0.7024,2.6251},
        {0.7292,2.6751},
        {0.7564,2.7251},
        {0.7842,2.7751},
        {0.8125,2.8251},
        {0.8412,2.8752},
        {0.8705,2.9252},
        {0.9002,2.9751},
        {0.9305,3.0251},
        {0.9613,3.0752},
        {0.9925,3.1252},
        {1.0242,3.1752},
        {1.0565,3.2251},
        {1.0891,3.2751},
        {1.1224,3.3251},
        {1.1562,3.3751},
        {1.1905,3.4252},
        {1.2253,3.4752},
        {1.2605,3.5252},
        {1.2963,3.5752},
        {1.3325,3.6252},
        {1.3692,3.6752},
        {1.4065,3.7251},
        {1.4442,3.7751},
        {1.4824,3.8251},
        {1.5212,3.8751},
        {1.5604,3.9251},
        {1.6002,3.9751},
        {1.6404,4.0251},
        {1.6812,4.0751},
        {1.7225,4.1251},
        {1.7642,4.1751},
        {1.8065,4.2251},
        {1.8492,4.2751},
        {1.8924,4.3251},
        {1.9362,4.3751},
        {1.9805,4.4251},
        {2.0253,4.4751},
        {2.0705,4.5251},
        {2.1162,4.5751},
        {2.1625,4.6251},
        {2.2093,4.6751},
        {2.2565,4.7251},
        {2.3042,4.7751},
        {2.3525,4.8251},
        {2.4013,4.8751},
        {2.4505,4.9251},
        {2.5003,4.9751},
        {2.5504,5.0251},
        {2.6011,5.075},
        {2.6524,5.125},
        {2.7042,5.1751},
        {2.7565,5.2251},
        {2.8093,5.2751},
        {2.8624,5.3251},
        {2.9162,5.3751},
        {2.9704,5.425},
        {3.0253,5.4751},
        {3.0804,5.5251},
        {3.1361,5.575},
        {3.1924,5.625},
        {3.2491,5.675},
        {3.3064,5.725},
        {3.3643,5.7751},
        {3.4224,5.8251},
        {3.4813,5.8751},
        {3.5404,5.9251},
        {3.6002,5.9751},
        {3.6605,6.0251},
        {3.7212,6.0751},
        {3.7825,6.1251},
        {3.8443,6.1751},
        {3.9064,6.2251},
        {3.9691,6.275},
        {4.0325,6.325},
        {4.0962,6.3751},
        {4.1604,6.4251},
        {4.2252,6.475},
        {4.2904,6.525},
        {4.3562,6.575},
        {4.4225,6.6251},
        {4.4891,6.675},
        {4.5564,6.725},
        {4.6243,6.7751},
        {4.6924,6.8251},
        {4.7612,6.875},
        {4.8305,6.9251},
        {4.9001,6.9751},
        {4.9705,7.025},
        {5.0412,7.0751},
        {5.1124,7.125},
        {5.1843,7.1751},
        {5.2564,7.2251},
        {5.3292,7.275},
        {5.4025,7.325},
        {5.4763,7.3751},
        {5.5505,7.4251},
        {5.6253,7.4751},
        {5.7004,7.5251},
        {5.7762,7.575},
        {5.8525,7.6251},
        {5.9293,7.6751},
        {6.0064,7.7251},
        {6.0842,7.775},
        {6.1624,7.825},
        {6.2412,7.875},
        {6.3205,7.9251},
        {6.4002,7.975},
        {6.4803,8},
        {6.5602,8},
        {6.6401,8},
        {6.7202,8},
        {6.8002,8},
        {6.8803,8},
        {6.9601,8},
        {7.0402,8},
        {7.1202,8},
        {7.2002,8},
        {7.2802,8},
        {7.3602,8},
        {7.4402,8},
        {7.5203,8},
        {7.6002,8},
        {7.6802,8},
        {7.7602,8},
        {7.8401,8},
        {7.9202,8},
        {8.0003,8},
        {8.0803,8},
        {8.1602,8},
        {8.2403,8},
        {8.3202,8},
        {8.4003,8},
        {8.4802,8},
        {8.5601,8},
        {8.6401,8},
        {8.7202,8},
        {8.8003,8},
        {8.8802,8},
        {8.9602,8},
        {9.0402,8},
        {9.1202,8},
        {9.2001,8},
        {9.2802,8},
        {9.3602,8},
        {9.4402,8},
        {9.5202,8},
        {9.6002,8},
        {9.6802,8},
        {9.7601,7.9988},
        {9.8399,7.964},
        {9.9189,7.914},
        {9.9976,7.864},
        {10.0758,7.8139},
        {10.1534,7.7639},
        {10.2306,7.714},
        {10.3073,7.6639},
        {10.3832,7.614},
        {10.4589,7.564},
        {10.534,7.514},
        {10.6088,7.464},
        {10.6829,7.4139},
        {10.7565,7.364},
        {10.8297,7.3139},
        {10.9023,7.264},
        {10.9744,7.214},
        {11.0461,7.164},
        {11.1172,7.114},
        {11.1878,7.064},
        {11.258,7.014},
        {11.3276,6.964},
        {11.3967,6.914},
        {11.4654,6.864},
        {11.5335,6.814},
        {11.6012,6.764},
        {11.6683,6.714},
        {11.7349,6.664},
        {11.8011,6.614},
        {11.8667,6.564},
        {11.9319,6.514},
        {11.9966,6.4639},
        {12.0606,6.4139},
        {12.1244,6.3639},
        {12.1875,6.3139},
        {12.25,6.264},
        {12.3122,6.214},
        {12.3738,6.164},
        {12.4351,6.1139},
        {12.4957,6.0639},
        {12.5559,6.0139},
        {12.6155,5.9639},
        {12.6746,5.9139},
        {12.7332,5.864},
        {12.7914,5.814},
        {12.849,5.7639},
        {12.9061,5.7139},
        {12.9627,5.664},
        {13.0189,5.614},
        {13.0746,5.5639},
        {13.1297,5.5139},
        {13.1843,5.464},
        {13.2385,5.414},
        {13.2922,5.3639},
        {13.3453,5.3139},
        {13.3979,5.2639},
        {13.45,5.214},
        {13.5017,5.164},
        {13.5528,5.114},
        {13.6034,5.064},
        {13.6537,5.0139},
        {13.7032,4.9639},
        {13.7524,4.9139},
        {13.801,4.8639},
        {13.8491,4.8139},
        {13.8969,4.7639},
        {13.9439,4.7139},
        {13.9906,4.664},
        {14.0367,4.6139},
        {14.0823,4.564},
        {14.1276,4.5139},
        {14.1721,4.4639},
        {14.2163,4.414},
        {14.2599,4.3639},
        {14.303,4.3139},
        {14.3456,4.264},
        {14.3879,4.214},
        {14.4296,4.1639},
        {14.4706,4.1139},
        {14.5112,4.064},
        {14.5514,4.014},
        {14.5911,3.9639},
        {14.6302,3.9139},
        {14.6688,3.864},
        {14.707,3.814},
        {14.7446,3.7639},
        {14.7818,3.7139},
        {14.8184,3.6639},
        {14.8545,3.6139},
        {14.8901,3.5639},
        {14.9254,3.5139},
        {14.9599,3.4639},
        {14.9942,3.4139},
        {15.0277,3.3639},
        {15.0609,3.3139},
        {15.0935,3.2639},
        {15.1257,3.2139},
        {15.1573,3.1638},
        {15.1885,3.1138},
        {15.2191,3.0639},
        {15.2492,3.0139},
        {15.2788,2.964},
        {15.3081,2.9139},
        {15.3366,2.8638},
        {15.3647,2.8139},
        {15.3925,2.7639},
        {15.4195,2.7139},
        {15.4463,2.6639},
        {15.4723,2.6138},
        {15.498,2.5638},
        {15.5231,2.5138},
        {15.5478,2.4639},
        {15.5719,2.4139},
        {15.5956,2.3639},
        {15.6186,2.3139},
        {15.6413,2.2639},
        {15.6634,2.2139},
        {15.6851,2.1639},
        {15.7062,2.1139},
        {15.7269,2.0639},
        {15.747,2.0138},
        {15.7667,1.9638},
        {15.7858,1.9137},
        {15.8044,1.8639},
        {15.8227,1.8138},
        {15.8403,1.7637},
        {15.8574,1.7137},
        {15.874,1.6638},
        {15.8901,1.6138},
        {15.9058,1.5638},
        {15.9209,1.5138},
        {15.9356,1.4637},
        {15.9498,1.4136},
        {15.9633,1.3638},
        {15.9765,1.3138},
        {15.9892,1.2636},
        {16.0013,1.2136},
        {16.0128,1.1638},
        {16.0241,1.1137},
        {16.0347,1.0635},
        {16.0448,1.0136},
        {16.0545,0.9636},
        {16.0636,0.9135},
        {16.0723,0.8634},
        {16.0803,0.8136},
        {16.088,0.7637},
        {16.0951,0.7137},
        {16.1018,0.6636},
        {16.1079,0.6134},
        {16.1135,0.5636},
        {16.1187,0.5137},
        {16.1233,0.4637},
        {16.1275,0.4134},
        {16.1312,0.3628},
        {16.1342,0.313},
        {16.137,0.2628},
        {16.139,0.2123},
        {16.1406,0.1632},
        {16.1419,0.1094},
        {16.1426,0.04}
    };

//feetR,velR  - velocity is feet/sec
const double auto16_R[][2] = {
        {0,0},
        {0.0004,0.0384},
        {0.0012,0.0768},
        {0.0025,0.128},
        {0.0042,0.1776},
        {0.0064,0.2262},
        {0.0092,0.2759},
        {0.0125,0.326},
        {0.0161,0.3756},
        {0.0205,0.4255},
        {0.0252,0.4758},
        {0.0304,0.5256},
        {0.0362,0.5757},
        {0.0424,0.6255},
        {0.0492,0.6755},
        {0.0565,0.7258},
        {0.0643,0.7757},
        {0.0724,0.8255},
        {0.0813,0.8755},
        {0.0905,0.9256},
        {0.1003,0.9756},
        {0.1104,1.0254},
        {0.1211,1.0751},
        {0.1325,1.1253},
        {0.1442,1.1753},
        {0.1565,1.2253},
        {0.1692,1.2754},
        {0.1825,1.3254},
        {0.1962,1.3753},
        {0.2104,1.4251},
        {0.2252,1.4751},
        {0.2405,1.5253},
        {0.2562,1.5754},
        {0.2724,1.6252},
        {0.2892,1.6752},
        {0.3065,1.7253},
        {0.3242,1.7753},
        {0.3424,1.8252},
        {0.3612,1.8752},
        {0.3805,1.9253},
        {0.4003,1.9753},
        {0.4205,2.0253},
        {0.4411,2.0751},
        {0.4625,2.1252},
        {0.4842,2.1752},
        {0.5064,2.225},
        {0.5292,2.2751},
        {0.5524,2.3251},
        {0.5761,2.3751},
        {0.6004,2.425},
        {0.6253,2.4752},
        {0.6504,2.5252},
        {0.6762,2.5751},
        {0.7024,2.6251},
        {0.7292,2.6751},
        {0.7564,2.7251},
        {0.7841,2.7751},
        {0.8125,2.8251},
        {0.8412,2.8752},
        {0.8705,2.9252},
        {0.9002,2.9751},
        {0.9305,3.0251},
        {0.9613,3.0752},
        {0.9925,3.1252},
        {1.0242,3.1752},
        {1.0565,3.2251},
        {1.0891,3.2751},
        {1.1224,3.325},
        {1.1562,3.3751},
        {1.1905,3.4252},
        {1.2253,3.4752},
        {1.2605,3.5252},
        {1.2963,3.5752},
        {1.3325,3.6252},
        {1.3692,3.6752},
        {1.4065,3.7251},
        {1.4442,3.7751},
        {1.4824,3.8251},
        {1.5212,3.8751},
        {1.5604,3.925},
        {1.6002,3.975},
        {1.6404,4.0251},
        {1.6812,4.0751},
        {1.7225,4.1251},
        {1.7642,4.1751},
        {1.8065,4.2251},
        {1.8492,4.2751},
        {1.8924,4.3251},
        {1.9362,4.3751},
        {1.9805,4.4251},
        {2.0253,4.4751},
        {2.0705,4.5251},
        {2.1162,4.5751},
        {2.1625,4.6251},
        {2.2093,4.6751},
        {2.2564,4.7251},
        {2.3042,4.7751},
        {2.3525,4.8251},
        {2.4013,4.8751},
        {2.4505,4.9251},
        {2.5002,4.9751},
        {2.5504,5.0251},
        {2.6011,5.075},
        {2.6524,5.125},
        {2.7042,5.1751},
        {2.7565,5.2251},
        {2.8092,5.2751},
        {2.8624,5.3251},
        {2.9162,5.3751},
        {2.9704,5.425},
        {3.0253,5.4751},
        {3.0804,5.5251},
        {3.1361,5.575},
        {3.1924,5.625},
        {3.2491,5.675},
        {3.3064,5.725},
        {3.3643,5.7751},
        {3.4224,5.8251},
        {3.4813,5.8751},
        {3.5404,5.9251},
        {3.6002,5.975},
        {3.6605,6.0251},
        {3.7212,6.0751},
        {3.7825,6.1251},
        {3.8443,6.1751},
        {3.9064,6.2251},
        {3.9691,6.275},
        {4.0325,6.325},
        {4.0962,6.3751},
        {4.1604,6.4251},
        {4.2252,6.475},
        {4.2904,6.525},
        {4.3562,6.575},
        {4.4225,6.6251},
        {4.4891,6.675},
        {4.5564,6.725},
        {4.6243,6.7751},
        {4.6924,6.8251},
        {4.7612,6.875},
        {4.8305,6.9251},
        {4.9001,6.975},
        {4.9705,7.025},
        {5.0412,7.0751},
        {5.1124,7.125},
        {5.1843,7.1751},
        {5.2564,7.2251},
        {5.3291,7.275},
        {5.4025,7.325},
        {5.4763,7.3751},
        {5.5505,7.4251},
        {5.6253,7.4751},
        {5.7004,7.5251},
        {5.7762,7.575},
        {5.8525,7.6251},
        {5.9293,7.6751},
        {6.0064,7.7251},
        {6.0842,7.775},
        {6.1624,7.825},
        {6.2412,7.875},
        {6.3205,7.9251},
        {6.4002,7.975},
        {6.4803,8},
        {6.5602,8},
        {6.6401,8},
        {6.7202,8},
        {6.8002,8},
        {6.8803,8},
        {6.9601,8},
        {7.0402,8},
        {7.1202,8},
        {7.2002,8},
        {7.2801,8},
        {7.3602,8},
        {7.4402,8},
        {7.5203,8},
        {7.6002,8},
        {7.6802,8},
        {7.7602,8},
        {7.8401,8},
        {7.9202,8},
        {8.0003,8},
        {8.0803,8},
        {8.1602,8},
        {8.2403,8},
        {8.3202,8},
        {8.4003,8},
        {8.4802,8},
        {8.5601,8},
        {8.6401,8},
        {8.7202,8},
        {8.8002,8},
        {8.8802,8},
        {8.9602,8},
        {9.0402,8},
        {9.1202,8},
        {9.2001,8},
        {9.2802,8},
        {9.3602,8},
        {9.4402,8},
        {9.5202,8},
        {9.6002,8},
        {9.6802,8},
        {9.7601,7.9988},
        {9.8398,7.964},
        {9.9189,7.914},
        {9.9976,7.864},
        {10.0758,7.8139},
        {10.1534,7.7639},
        {10.2306,7.714},
        {10.3072,7.6639},
        {10.3832,7.614},
        {10.4589,7.564},
        {10.534,7.514},
        {10.6088,7.464},
        {10.6829,7.4139},
        {10.7565,7.364},
        {10.8297,7.3139},
        {10.9023,7.264},
        {10.9744,7.214},
        {11.0461,7.164},
        {11.1172,7.114},
        {11.1878,7.064},
        {11.258,7.014},
        {11.3276,6.964},
        {11.3967,6.914},
        {11.4654,6.864},
        {11.5335,6.814},
        {11.6012,6.764},
        {11.6683,6.714},
        {11.7349,6.664},
        {11.8011,6.614},
        {11.8667,6.564},
        {11.9319,6.514},
        {11.9966,6.4639},
        {12.0606,6.414},
        {12.1244,6.364},
        {12.1875,6.3139},
        {12.25,6.264},
        {12.3122,6.214},
        {12.3738,6.164},
        {12.4351,6.1139},
        {12.4957,6.0639},
        {12.5559,6.0139},
        {12.6155,5.9639},
        {12.6746,5.9139},
        {12.7332,5.864},
        {12.7914,5.814},
        {12.849,5.7639},
        {12.9061,5.7139},
        {12.9627,5.664},
        {13.0189,5.614},
        {13.0746,5.5639},
        {13.1297,5.5139},
        {13.1843,5.464},
        {13.2385,5.414},
        {13.2922,5.3639},
        {13.3453,5.3139},
        {13.3979,5.2639},
        {13.45,5.214},
        {13.5017,5.164},
        {13.5528,5.114},
        {13.6034,5.064},
        {13.6537,5.0139},
        {13.7032,4.9639},
        {13.7524,4.914},
        {13.801,4.8639},
        {13.8491,4.814},
        {13.8969,4.7639},
        {13.9439,4.7139},
        {13.9906,4.664},
        {14.0367,4.6139},
        {14.0823,4.564},
        {14.1276,4.5139},
        {14.1721,4.4639},
        {14.2163,4.414},
        {14.2599,4.3639},
        {14.303,4.3139},
        {14.3456,4.264},
        {14.3879,4.214},
        {14.4296,4.1639},
        {14.4706,4.1139},
        {14.5112,4.064},
        {14.5514,4.014},
        {14.5911,3.9639},
        {14.6302,3.9139},
        {14.6688,3.864},
        {14.707,3.814},
        {14.7446,3.7639},
        {14.7818,3.7139},
        {14.8184,3.6639},
        {14.8545,3.6139},
        {14.8901,3.5639},
        {14.9254,3.5139},
        {14.9599,3.4639},
        {14.9942,3.4139},
        {15.0277,3.3639},
        {15.0609,3.3139},
        {15.0935,3.2639},
        {15.1257,3.2139},
        {15.1573,3.1638},
        {15.1885,3.1138},
        {15.2191,3.0639},
        {15.2492,3.0139},
        {15.2788,2.964},
        {15.3081,2.9139},
        {15.3366,2.8638},
        {15.3647,2.8139},
        {15.3925,2.7639},
        {15.4195,2.7139},
        {15.4463,2.6639},
        {15.4723,2.6138},
        {15.498,2.5638},
        {15.5231,2.5138},
        {15.5478,2.4639},
        {15.5719,2.4139},
        {15.5956,2.3639},
        {15.6186,2.3139},
        {15.6413,2.2639},
        {15.6634,2.2139},
        {15.6851,2.1639},
        {15.7062,2.1139},
        {15.7269,2.0639},
        {15.747,2.0138},
        {15.7667,1.9638},
        {15.7858,1.9137},
        {15.8044,1.8639},
        {15.8227,1.8138},
        {15.8403,1.7637},
        {15.8574,1.7137},
        {15.874,1.6638},
        {15.8901,1.6138},
        {15.9058,1.5638},
        {15.9209,1.5138},
        {15.9356,1.4637},
        {15.9498,1.4136},
        {15.9633,1.3638},
        {15.9765,1.3138},
        {15.9892,1.2636},
        {16.0013,1.2136},
        {16.0128,1.1638},
        {16.0241,1.1137},
        {16.0347,1.0635},
        {16.0448,1.0136},
        {16.0545,0.9636},
        {16.0636,0.9135},
        {16.0723,0.8634},
        {16.0803,0.8136},
        {16.088,0.7637},
        {16.0951,0.7137},
        {16.1018,0.6636},
        {16.1079,0.6134},
        {16.1135,0.5636},
        {16.1187,0.5137},
        {16.1233,0.4637},
        {16.1275,0.4134},
        {16.1312,0.3628},
        {16.1342,0.313},
        {16.137,0.2628},
        {16.139,0.2123},
        {16.1406,0.1632},
        {16.1419,0.1094},
        {16.1426,0.04}
    }; 

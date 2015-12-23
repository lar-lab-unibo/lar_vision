points=[
    261035
    522070
    1044140
    2088280
    4176560
    8353120
    16706240
    33412480
    66824960
]

ransac=[
    41
    87
    171
    324
    646
    1292
    2676
    5417
    12001
];

onepoint=[
    20
    41
    82
    165
    349
    681
    1434
    2637
    5851  
];


heightmap1x=[
    261035,19
    522070,35
    1044140,70
    2088280,140
    4176560,270
    8353120,547
    16706240,1109
    33412480,2202
    66824960,4491  
];


heightmap5x=[
    261035,9
    522070,18
    1044140,54
    2088280,91
    4176560,175
    8353120,343
    16706240,657
    33412480,1319
    66824960,2627
];

plot(points,ransac,'r', 'LineWidth',2);hold on;
plot(points,onepoint,'b', 'LineWidth',2);hold on;
plot(points,heightmap1x(:,2),'g', 'LineWidth',2);hold on;
plot(points,heightmap5x(:,2),'g--', 'LineWidth',2);hold on;

%plot(r_reference,r_vader*1.2,'g', 'LineWidth',2);hold on;
%plot(r_reference,r_toycar,'b', 'LineWidth',2);hold on;
legend('Ransac Plane Model','1-Point Ransac','HeightMap 1x','HeightMap 5x')
hold on;
xlabel('Cloud Points');
ylabel('Milliseconds');
grid on;
    
# 引言
现在有许多的基于Si5351的ft8自制收发信机，比如ADX，AFP-FSK。但是这些都只是工作在短波段。  
没有一个能工作在6m波段，而6m波段是一个有趣的魔术波段，很容易低功率传播很远。  

这是因为Si5351无法直接产生6m（50.313Mhz）波段的ft8信号，而使用混频的方法，会让电路变的非常复杂。  
所以我采用了二次谐波的方法间接产生6m（50.313Mhz）波段的ft8信号。  

只需要修改  
#define FT8_TONE_SPACING       312 //625          // ~3.12 Hz     //产生二次谐波，就可以到6.24hz  
#define FT8_DEFAULT_FREQ        25156500UL   //产生二次谐波，就可以到50.313Mhz  

再使用50Mhz带通滤波器选择出信号，就可以产生6m波段的ft8信号。

再通过一个功放部分，就可以组成一个最简单的6m波段发射机。


我的呼号是BI6LTD，期待空中通联。如果有需要，可以通过[QRZ主页](https://www.qrz.com/db/BI6LTD)联系我


参考项目  
https://github.com/kholia/Easy-FT8-Beacon-v3/blob/master/Easy-FT8-Beacon-v3/Easy-FT8-Beacon-v3.ino  
https://github.com/etherkit/Si5351Arduino  

# DIY SCARA robot arm on STM
**![](https://lh4.googleusercontent.com/ABQd5vq1c0ldNSoEnwkOGhyIGZaqVQTFeKYxtfsNj3kGQC6vxOHLKNlYYT2ozuKuFk2zLqL6891QtyBnZOXcPDm5re1l9ceZ-LM_snvF2MmU51Ttq4sJ8PyVP-bOtimQ9T2eR49-wyhFdd7ZgCoX0w)**
This was my engineering project for my degree. Some diagrams are still in Polish, I need to update them in the future. I managed to asseble this arm and it even worked! Still there is still some improvment to make, like changing step motors. I have some problems with them in the past, because the motors sometimes lost the position. I think It will be beter to use servo motors or encoders.

## The project is divided into 3 parts:

 - 3D files that you can print or manufacture in any other way. There are .stl files for printing and .ipt files. 
 - Matlab GUI for control and other scripts,
 - Source code for embedded system
## Used tools
-   Autodesk Inventor, 
-   Solid Works,
-   Ultimeaker Cura,
-   STM32 Cube IDE,
-   STMStudio,
-   Matlab,

## Hardware used
- drivers [DRV8825](https://www.pololu.com/product/2133/resources),
- step motors (some used motors from fax machine :) ),
- microcontroller [STM32F411RE ARM Cortex M4](https://www.st.com/en/microcontrollers-microprocessors/stm32f411re.html),
- impulse power supply (I use power supply for led strips, like [this](https://duckduckgo.com/?q=led+strip+power+supply&t=brave&iax=images&ia=images)),

## Electronic connection diagram
**![](https://lh5.googleusercontent.com/nKmWJya0fbPWllSuQG_HQbM3DfYyDt-D9mA5hlWi7UDCMJye_BiMH2IU7uxxWnVfjyAYpFUSWzN58-nSRfCej1MkS4L7Yngdn_mJDnEYBA_8KRHLEEIubmvhMm2AaKCJHVUYA3jSzPIqjDf9Sx4xeQ)**
## Configuration for microcontroller (need to update for english)
**![](https://lh4.googleusercontent.com/QScvlhmxzFa8vF1dAop5HeOpV6ow-eaoBgH1uNNaFO0UOSsm7i3ywd0YcP1Q7tkSM3ZkUmGWNNy9b9XqyT-ITDF197DPwa0yyejYGSJsqh1icJW07QhE2957qBZ6F08p8xflGptnkLFiwM1TbsX3cw)**
# Diagram how the microcontroller works (need to update for english)
**![](https://lh3.googleusercontent.com/V1Qc-ojZLnrgzsDHpTgU1ikWuBvMBiQC-pDYRW6ycB7fvoWEoxNiN43ZXLz99xh0iM9ovNDyTPZgKvXOpyxSm6ZN-d7WVnCRn_LeQaU2c-Qsek9Bqhq5bylSB3r0jv1Rgq4Z9rBSYhVFmuTvk1irJA)**
**![](https://lh4.googleusercontent.com/0ht6VHCupL8t4bBERGWUAZHz_y4jcy46S67kHKJvGChkQaMTBez2xoHmYMnqdsiuZMixcJe5aP-oQoxWEdga1gCYPUvwhI5727jpGE1EhffibXkIKN5SdF_Ct_lDy2_ndUVdEN59L4Ygsig5TgKlOw)**
## Diagram how GUI works (need to update for english)
**![](https://lh3.googleusercontent.com/9ME3EkPGCfWSMHXNhlisFo4qE9BVxlJ-WTjOTdNCgGWW3hZemoOB0p6WUqxd7G4aynbeX0L3QdBRMtybPCmFn3U6dRyHJ9CEger93IjJt47axH4ZgrmY7zI_WTk48BT3iiHdO6hzO-8uSihuJoZUjQ)**

# C-Plus-Plus Beispiel mit STM32F030

STM32F030FP4:
* 16kByte Flash
*  4kByte RAM

src/Beispiel.cpp enthält die main Funktion.
src/Hardware.cpp enthält die Interrupt-Vektor-Tabelle, sowie alle leeren Interrupts.
Außerdem enthält sie auch eine grundlegende Initialisierung des Controllers. Weiterhin ist dort der ResetHandler implementiert.
src/timer.cpp implementiert den SysTick_Handler sowie eine grundlegende Software-Timer-Funktion.
Unter Drivers liegen alle ST HAL/LL Dateien, welche in diesem Beispiel verwendet werden.

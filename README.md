# Battery-Evaluator
A comprehensive battery evaluator board for running simple tests and measurements, or creating a battery profile for accurately predicting charge level from voltage.

![](https://github.com/AdinAck/Wiki-Images/blob/main/Battery-Evaluator/gui.png?raw=true)

![](https://github.com/AdinAck/Wiki-Images/blob/main/Battery-Evaluator/3d.png?raw=true)

![](https://github.com/AdinAck/Wiki-Images/blob/main/Battery-Evaluator/real.jpg?raw=true)

## Electrical Characteristics
- 0-5v battery (single cell)
- 0-1000mA programable constant current draw
- Do not reverse polarity of battery, the board will die
- While you *can* plug in a battery before USB is plugged in... you shouldn't

## Software Features
- Graph the measured and predicted battery voltage with respect to mAh consumed
- Measure ESR
- Measure mWh consumed
- Monitor temperature of FETs and resistors

## To Be Implemented
- Constant power discharge mode
- More battery chemistries
- Custom battery voltage range
- [Op-Amp oscillation fix](https://github.com/AdinAck/Battery-Evaluator/issues/1)

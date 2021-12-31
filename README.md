# BiRiskRRTValidation

### _*.py_ Files Explanation

* _riksrrt.py_
  * Implement unidirectional RiskRRT. 

* _biriksrrt.py_
  * Implement bidirectional RiskRRT (BiRiskRRT) and endow RiskRRT and BiRiskRRT with dynamic planning capability.

* _param.py_
  * setup hyper-parameters for the planners.

* _utils.py_
  * define the data structures and classes used in the planners.

* _main.py_
  * run the planners multiple times and record the statistic results into 'results' folder.

* _result_analysis.py_
  * compute and print the mean values and standard values of the trails conducted by _main.py_.
***************************
### Folders Introduction

* ___/data___
  * This folder stores the data of dynamic pedestrians from different datasets.

* ___/maps___
  * This folder contains the test maps (_.png_ format).

* ___results___
  * This folder stores the test results and also the visualizations of the planning processes.
  * ___/dynamic___ is for tests with dynamic obstacles (pedestrians).
  * ___/static___ is for tests without dynamic obstacles.

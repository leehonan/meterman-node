# Change Log

Release/revision numbering is major only, no minors - i.e. 1,2,3... not 1.1, 1.2, 2.0, etc.

#### 2017-11-27 R6
* Baseline release to GitHub (from private repo).

#### 2017-11-29 R7
* Changed CT clamp to be enabled/disabled from command line.  
* Various optimisations to cram program into ROM (mostly shortening strings).

#### 2018-01-22 R8
* Changed sscanf calls to use SCNu8 macro for uint_8t vars, SCNd8 macro for int_8t vars

#### 2018-02-06 R9
* Reduced default modem baud rate to 9.6kbps
* Abbreviated some console text to fit MCU when compiled with high power enabled

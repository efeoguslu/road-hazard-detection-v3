set terminal pngcairo size 800,600
set output 'output.png'
set title 'My Plot Title'
set xlabel 'X-Axis Label'
set ylabel 'Y-Axis Label'
plot 'compoundVectorFilteredLogFile.txt' using 1 with linespoints
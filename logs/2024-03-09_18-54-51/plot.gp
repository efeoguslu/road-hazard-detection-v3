set title 'Sensor Data Comparison'
set xlabel 'Time'
set ylabel 'Value'
set terminal png
set output 'sensor_data_comparison.png'
plot 'axLogFile.txt' with lines title 'Sensor 1', \n'axFilteredLogFile.txt' with lines title 'Sensor 2'

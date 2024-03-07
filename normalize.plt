# Set plot properties
set xlabel "Sample"
set ylabel "Value"
set title "Sensor Data"

# Set delimiter to comma
set datafile separator ","

# Plot calibrated data
plot "logs/2024-03-07_12-17-38_normalized_accel_log.txt" using 1 with lines title "ax", \
       "" using 2 with lines title "ay", \
       "" using 3 with lines title "az"

replot "logs/2024-03-07_12-22-02_sensor_data_log.txt" using 2 with lines title "ax", \
       "" using 3 with lines title "ay", \
       "" using 4 with lines title "az"




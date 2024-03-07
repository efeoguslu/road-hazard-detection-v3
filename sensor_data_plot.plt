# Set plot properties
set xlabel "Sample"
set ylabel "Value"
set title "Sensor Data"

# Set delimiter to comma
set datafile separator ","

# Plot calibrated data
plot "logs/2024-03-05_11-39-46_sensor_data_log.txt" using 2 with lines title "ax", \
       "" using 3 with lines title "ay", \
       "" using 4 with lines title "az", \
       "" using 5 with lines title "gr", \
       "" using 6 with lines title "gp", \
       "" using 7 with lines title "gy"
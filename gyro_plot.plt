# Set plot properties
set xlabel "Sample Number"
set ylabel "Sensor Value"
set title "Sensor Gyroscope Data"

# Set delimiter to comma
set datafile separator ","

# Plot calibrated data
plot "2024-03-05_11-39-46_sensor_data_log.txt" using 5 with lines title "gr", \
       "" using 6 with lines title "gp", \
       "" using 7 with lines title "gy"
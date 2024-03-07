# Set plot properties
set xlabel "Sample Number"
set ylabel "Sensor Value"
set title "Sensor Acceleration Data"

# Set delimiter to comma
set datafile separator ","

# Plot calibrated data
plot "logs/2024-03-05_11-39-46_sensor_data_log.txt" using 2 with lines title "ax", \
       "" using 3 with lines title "ay", \
       "" using 4 with lines title "az"
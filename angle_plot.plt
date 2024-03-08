# Set plot properties
set xlabel "Sample Number"
set ylabel "Sensor Value"
set title "Sensor Angle Values"

# Set delimiter to comma
set datafile separator ","

# Plot calibrated data
plot "logs/2024-03-08_10-45-08_angles_log.txt" using 1 with lines title "roll (deg)", \
       "" using 2 with lines title "pitch (deg)"
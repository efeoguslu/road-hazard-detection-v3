# Set plot properties
set xlabel "Sample"
set ylabel "Value"
set title "Sensor Data"

# Set delimiter to comma
set datafile separator ','

# Plot calibrated data
plot "anglesLogFile.txt" using 1 with lines title "roll", \
       "" using 2 with lines title "pitch"

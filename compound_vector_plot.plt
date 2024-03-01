# Set plot properties
set xlabel "Sample"
set ylabel "Value"
set title "Sensor Data"

# Set delimiter to comma
set datafile separator ","

# Plot calibrated data
plot "compound_vector.txt" using 2 with lines title "CAV", \
       "" using 3 with lines title "CGV"
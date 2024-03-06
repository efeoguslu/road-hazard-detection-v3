# Set plot properties
set xlabel "Sample"
set ylabel "Value"
set title "Sensor Data"

# Set delimiter to comma
set datafile separator ","

# Plot calibrated data
plot "2024-03-05_11-39-46_compound_vector.txt" using 2 with lines title "CAV", \
       "" using 3 with lines title "CGV"
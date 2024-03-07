# Set plot properties
set xlabel "Sample Number"
set ylabel "Vector Magnitude"
set title "Compound Gyroscope Vector"

# Set delimiter to comma
set datafile separator ","

# Plot calibrated data
plot "logs/2024-03-05_11-39-46_compound_vector.txt" using 3 with lines title "CGV"
# Set the title of the plot
set title "Sensor Data Comparison"

# Set labels for x and y axes
set xlabel "Time"
set ylabel "Value"

# Set the output file format and name
set terminal png
set output "sensor_data_comparison.png"

# Plot the data from the first .txt file
plot "axLogFile.txt" with lines title "Sensor 1", \
     "axFilteredLogFile.txt" with lines title "Sensor 2"
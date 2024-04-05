# Set plot title and axis labels
set title "Sensor Data Plot"
set xlabel "Sample Index"
set ylabel "Values"

# Plot data from the file as a line graph
plot "sensorLogFileWithoutText.txt" using 1 with lines title "My Data"
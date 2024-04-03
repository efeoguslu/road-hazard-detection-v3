# Set plot title and axis labels
set title "Sensor Data Plot"
set xlabel "Sample Index"
set ylabel "Values"

# Set data file
set datafile separator ','

# Plot data from file as a line graph
plot "sensorLogFile.txt" using 0:2 with lines title "ax", \
     "" using 0:3 with lines title "ay", \
     "" using 0:4 with lines title "az"
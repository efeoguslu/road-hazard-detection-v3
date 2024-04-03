# Set plot title and axis labels
set title "Data Plot"
set xlabel "Index"
set ylabel "Value"

# Set data file
set datafile separator ','

# Set y-axis limits
set yrange [0:2]

# Plot data from file as a line graph
plot "rotatedAzLogFile.txt" using 0:2 with lines title "rotatedAzLogFile", \
     "iirFilter.txt" using 0:2 with lines title "iirFilterOutput"

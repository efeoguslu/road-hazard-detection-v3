# Set plot title and axis labels
set title "Data Plot"
set xlabel "Index"
set ylabel "Value"

# Set data file
set datafile separator ','

# Set y-axis limits
set yrange [0:2]

# Plot data from file as a line graph
plot "meanLogFile.txt" using 0:2 with lines title "meanLogFile", \
     "standartDeviationLogFile.txt" using 0:2 with lines title "standartDeviationLogFile", \
     "varianceLogFile.txt" using 0:2 with lines title "varianceLogFile"
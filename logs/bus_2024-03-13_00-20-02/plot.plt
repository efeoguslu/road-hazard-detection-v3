# Set the title of the plot
set title "Line Graph of Data"

# Set the x-axis label
set xlabel "Index"

# Set the y-axis label
set ylabel "Value"

# Plot the data from the file
plot 'axLogFile.txt' using 0:1 with lines title "Data"
replot 'ayLogFile.txt' using 0:1 with lines
replot 'azLogFile.txt' using 0:1 with lines
replot 'azFilteredLogFile.txt' using 0:1 with lines

        # Set plot title and axis labels
        set title "Sensor Data Plot"
        set xlabel "Sample Index"
        set ylabel "Values"

        # Set data file
        set datafile separator ','

        # Set y-axis limits
        # set yrange [0:2]

        # Plot data from file as a line graph
        plot "rotatedAzLogFile.txt" using 0:2 with lines title "rotatedAzLogFile", \
            "iirFilter.txt" using 0:2 with lines title "iirFilterOutput", \
            "standartDeviationLogFile.txt" using 0:2 with lines title "standartDeviationLogFile"
        
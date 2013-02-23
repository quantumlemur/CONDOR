import os
import csv


folder = 'Output/ConsolidatedDesignSpaceFiles/'

outputFileName = folder + 'ConsolidatedDesignSpace_temp.csv'
finalOutputFileName = folder + 'designSpace_ALL.csv'

keys = []
if os.path.isfile(finalOutputFileName):
    with open(finalOutputFileName, 'rb') as f:
        reader = csv.DictReader(f)
        keys = sorted(reader.fieldnames)
    files = os.listdir(folder)
    with open(outputFileName, 'wb') as f:
        writer = csv.DictWriter(f, keys, delimiter=',')
        writer.writerow({key:key for key in keys})
        for inputFile in files:
            if inputFile.startswith('designSpace_') and inputFile.endswith('.csv'):
                print(inputFile)
                with open(folder + inputFile, 'rb') as openCsvFile:
                    reader = csv.DictReader(openCsvFile)
                    for row in reader:
                        writer.writerow(row)
                os.remove(folder + inputFile)
    os.rename(outputFileName, finalOutputFileName)
else:
    print('Give me a designSpace_ALL.csv file to get the headers out of!')
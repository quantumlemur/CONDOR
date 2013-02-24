import os
import csv
from shutil import copy


folder = 'Output/ConsolidatedDesignSpaceFiles/'

outputFileName = folder + 'ConsolidatedDesignSpace_temp.csv'
finalOutputFileName = folder + 'designSpace_ALL.csv'


keys = []
numKeys = 0
if os.path.isfile(finalOutputFileName):
    # make backup of complete design space file
    fnum = 0
    fileNameTemplate = 'Output/ConsolidatedDesignSpaceFiles/BACKUPdesignSpace_%d.csv'
    fileName = fileNameTemplate % (fnum)
    while os.path.isfile(fileName):
        fnum += 1
        fileName = fileNameTemplate % (fnum)
    copy(finalOutputFileName, fileName)
    with open(finalOutputFileName, 'rb') as f:
        reader = csv.DictReader(f)
        keys = sorted(reader.fieldnames)
        numkeys = len(keys)
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
                        if len(row) == numKeys:
                            writer.writerow(row)
                        else:
                            print 'Skipping incomplete line in %s' % inputFile
                os.remove(folder + inputFile)
    os.rename(outputFileName, finalOutputFileName)
else:
    print('Give me a designSpace_ALL.csv file to get the headers out of!')
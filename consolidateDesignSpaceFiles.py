# CONDOR
# Copyright (C) 2013 Michael Roberts

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.


import os
import csv
from shutil import copy


folder = 'Output/ConsolidatedDesignSpaceFiles/'

outputFileName = folder + 'ConsolidatedDesignSpace_temp.csv'
finalOutputFileName = folder + 'designSpace_ALL'


keys = []
numKeys = 0
if os.path.isfile(finalOutputFileName+'.csv'):
    # make backup of complete design space file
    fnum = 0
    fileNameTemplate = 'Output/ConsolidatedDesignSpaceFiles/BACKUPdesignSpace_%d.csv'
    fileName = fileNameTemplate % (fnum)
    while os.path.isfile(fileName):
        fnum += 1
        fileName = fileNameTemplate % (fnum)
    copy(finalOutputFileName+'.csv', fileName)
    with open(finalOutputFileName+'.csv', 'rb') as f:
        reader = csv.DictReader(f)
        keys = sorted(reader.fieldnames)
        numKeys = len(keys)
    files = os.listdir(folder)
    fall = open(folder + 'WithWing_ALL.csv', 'wb')
    f1 = open(folder + 'WithWing 1rotor 0aux.csv', 'wb')
    f2 = open(folder + 'WithWing 1rotor 1aux.csv', 'wb')
    f3 = open(folder + 'WithWing 2rotor 0aux.csv', 'wb')
    f4 = open(folder + 'WithWing 2rotor 1aux.csv', 'wb')
    writerall = csv.DictWriter(fall, keys, delimiter=',')
    writer1 = csv.DictWriter(f1, keys, delimiter=',')
    writer2 = csv.DictWriter(f2, keys, delimiter=',')
    writer3 = csv.DictWriter(f3, keys, delimiter=',')
    writer4 = csv.DictWriter(f4, keys, delimiter=',')
    writerall.writerow({key:key for key in keys})
    writer1.writerow({key:key for key in keys})
    writer2.writerow({key:key for key in keys})
    writer3.writerow({key:key for key in keys})
    writer4.writerow({key:key for key in keys})
    for inputFile in files:
        if inputFile.startswith('designSpace_') and inputFile.endswith('.csv'):
            print(inputFile)
            with open(folder + inputFile, 'rb') as openCsvFile:
                reader = csv.DictReader(openCsvFile)
                for row in reader:
                    if '' in row.values():
                        print 'Skipping incomplete line in %s' % inputFile
                    else:
                        writerall.writerow(row)
                        if row['NumRotors'] == '1':
                            if row['NumAuxProps'] == '0':
                                writer1.writerow(row)
                            else:
                                writer2.writerow(row)
                        else:
                            if row['NumAuxProps'] == '0':
                                writer3.writerow(row)
                            else:
                                writer4.writerow(row)
            os.remove(folder + inputFile)
    f1.close()
    f2.close()
    f3.close()
    f4.close()
    #os.rename(outputFileName, finalOutputFileName)
else:
    print('Give me a designSpace_ALL.csv file to get the headers out of!')
#%%
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: adinale
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Aiko Dinale
        Sim Bamford
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
importExportBatches... is for importing timestamped address-event data, 
and then exporting it again possibly in a different format. 
Intended for use with large files, it allows you to read in a certain amount 
of data in a batch, in order not to exceed the limits of working memory.
At the time of writing there is only a function:
    
- importExportBatchesIitYarp
    
Ideally we will extend this to handle all formats.     

Use params numberBatches and importMaxBytes for batch control. the info field
in the imported dict contains running info about the state of the import, 
but this in not encoded in the output file.
"""

import os
from .importIitYarp import importIitYarp
from .exportIitYarp import exportIitYarp
from .timestamps import offsetTimestampsForAContainer

def importExportBatchesIitYarp(datasetPath, numberBatches, importMaxBytes = 1000000, tsBits = 30):
    """
    Import a binary dataset into batches, then export each single batch into YARP format.

    Args:
        datasetPath (string): path containing the dataset in binary format, i.e. binaryevents.log
        numberBatches (int): number of batches which the input dataset should be split into
        importMaxBytes (int, optional): maximum numbe of Bytes to be imported. Defaults to 1000000 (1 MB).
    """    
    dataSizeBytes = os.path.getsize(datasetPath + "/binaryevents.log")

    numMegaByte = round(dataSizeBytes/importMaxBytes)
    sizeBatchMegaByte = round(numMegaByte/numberBatches)
    importedToByte = -1

    for idx in range(0, numberBatches):
        wrapOffset = 0
        for megaByteCounter in range(0, sizeBatchMegaByte):

            importedBatch = importIitYarp(filePathOrName = datasetPath,
                                        tsBits = tsBits,
                                        convertSamplesToImu = False,
                                        importFromByte = importedToByte + 1,
                                        importMaxBytes = importMaxBytes)

            if megaByteCounter == 0:
                # The first batch is treated differently
                exportIitYarp(importedBatch,
                            exportFilePath = os.path.join(datasetPath + "_convertedBatch" + str(idx)),
                            pathForPlayback =  os.path.join(datasetPath + "_convertedBatch" + str(idx)),
                            dataTypes = ['sample', 'dvs'],
                            protectedWrite = False,
                            writeMode = 'w')
                first_tsOffset = importedBatch['info']['tsOffsetFromData']
                previous_tsOffset = first_tsOffset
            else:
                # Offset timestamps in the second batch
                imported_tsOffset = importedBatch['info']['tsOffsetFromData']
                if imported_tsOffset > previous_tsOffset:
                    wrapOffset += 2**tsBits * 0.08 / 1000000

                offsetToApplyToImportedBatch = first_tsOffset - imported_tsOffset + wrapOffset
                offsetTimestampsForAContainer(importedBatch, offsetToApplyToImportedBatch)
                importedBatch['info']['tsOffsetFromData'] += offsetToApplyToImportedBatch

                exportIitYarp(importedBatch,
                            exportFilePath = os.path.join(datasetPath + "_convertedBatch" + str(idx)),
                            pathForPlayback =  os.path.join(datasetPath + "_convertedBatch" + str(idx)),
                            dataTypes = ['sample', 'dvs'],
                            protectedWrite = False,
                            writeMode = 'a')

                previous_tsOffset = imported_tsOffset

            time = importedBatch['data']['right']['dvs']['ts'][-1]
            print("Time " + str(time))

            importedToByte = importedBatch['info']['importedToByte']
            print(str(megaByteCounter) + ": imported to Byte " + str(importedToByte) + "\n")

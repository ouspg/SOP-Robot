import os
import random

LIBRISPEECH_PATH = ""

def getLength(elem):
    return elem[2]

def getListOfFlac(dirName):
    # create a list of file and sub directories 
    # names in the given directory 
    listOfFile = os.listdir(dirName)
    allFiles = list()
    # Iterate over all the entries
    for entry in listOfFile:
        # Create full path
        fullPath = os.path.join(dirName, entry)
        # If entry is a directory then get the list of files in this directory 
        if os.path.isdir(fullPath):
            allFiles = allFiles + getListOfFlac(fullPath)
        elif fullPath.endswith(".flac"):
            allFiles.append(fullPath)
                
    return allFiles

def pairSpeakers(file):
    with open(file) as s:
        lines = s.readlines()

    speakers = []
    for line in lines:
        line = line.strip().split('|')
        speakers.append([line[0].strip(), line[1].strip(), float(line[3].strip())])

    speakers.sort(key=getLength)

    pairs = []
    for i in range(0, len(speakers)-1, 2):
        pairs.append((speakers[i][0], speakers[i+1][0]))

    return pairs

def pairFiles(speakers, type):
    taskfile = open("{0}/{1}/task.txt".format(LIBRISPEECH_PATH, type), 'a')

    files1 = getListOfFlac("{0}/{1}/{2}".format(LIBRISPEECH_PATH, type, speakers[0]))
    files2 = getListOfFlac("{0}/{1}/{2}".format(LIBRISPEECH_PATH, type, speakers[1]))

    max_file_count = max(len(files1), len(files2))
    min_file_count = min(len(files1), len(files2))

    for i in range(0, min_file_count):
        taskfile.write("{0} {1}\r\n".format(files1[i], files2[i]))

    if max_file_count > min_file_count:
        max_src = 1 if(len(files1) > len(files2)) else 2

        for i in range(min_file_count, max_file_count):
            if max_src == 1:
                taskfile.write("{0} {1}\r\n".format(files1[i], random.choice(files2)))
            else:
                taskfile.write("{0} {1}\r\n".format(random.choice(files1), files2[i]))


def main():
    #clear previous task files
    data_types = ['train-clean-100', 'dev', 'test']
    for d in data_types:
        open("{0}/{1}/task.txt".format(LIBRISPEECH_PATH, d), 'w').close()

    #training
    pairs_tr = pairSpeakers('{0}/SPEAKERS_TR.TXT'.format(LIBRISPEECH_PATH))
    for pair in pairs_tr:
        pairFiles(pair, data_types[0])
    #dev
    pairs_cv = pairSpeakers('{0}/SPEAKERS-CV.TXT'.format(LIBRISPEECH_PATH))
    for pair in pairs_cv:
        pairFiles(pair, data_types[1])
    #test
    pairs_tt = pairSpeakers('{0}/SPEAKERS-TT.TXT'.format(LIBRISPEECH_PATH))
    for pair in pairs_tt:
        pairFiles(pair, data_types[2])

main()
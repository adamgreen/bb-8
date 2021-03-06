/*  Copyright (C) 2015  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "try_catch.h"


static void displayUsage(void)
{
    fprintf(stderr, "Usage: bin2h input.bin output.h variableName\n"
                    "Where: input.bin is the name of the input .bin file to be\n"
                    "                 converted into a hexadecimal .h header\n"
                    "                 file.\n"
                    "       output.h is the name of the output .h header file\n"
                    "                to be created.\n"
                    "       variableName is the name of the global variable to\n"
                    "                    be assigned the value of the binary\n"
                    "                    data in the header file.\n");
}


typedef struct BinaryFile
{
    uint8_t* pBinary;
    uint32_t binarySize;
} BinaryFile;


static BinaryFile openBinaryFile(const char* binaryFilename);
static void closeBinaryFile(BinaryFile* pFile);


int main(int argc, const char** argv)
{
    int         returnValue = 0;
    const char* pInputFilename = NULL;
    const char* pOutputFilename = NULL;
    const char* pVariableName = NULL;
    BinaryFile  inputFile = {NULL, 0};
    FILE*       pOutputFile = NULL;
    uint32_t    bytesLeft = 0;
    uint8_t*    pCurr = NULL;

    /* Parse command line parameters. */
    if (argc != 4)
    {
        displayUsage();
        return -1;
    }
    pInputFilename = argv[1];
    pOutputFilename = argv[2];
    pVariableName = argv[3];

    __try
    {
        inputFile = openBinaryFile(pInputFilename);
        pOutputFile = fopen(pOutputFilename, "w");
        if (!pOutputFile)
        {
            fprintf(stderr, "error: Failed to open %s for writing: ", pOutputFilename);
            perror(NULL);
            __throw(fileException);
        }

        fprintf(pOutputFile, "/* Autogenerated Header File */\n");
        fprintf(pOutputFile, "/*   bin2h %s %s %s */\n\n", argv[1], argv[2], argv[3]);
        fprintf(pOutputFile, "#include <stdint.h>\n\n");
        fprintf(pOutputFile, "uint8_t %s[] = \n", pVariableName);
        fprintf(pOutputFile, "{\n");

        bytesLeft = inputFile.binarySize;
        pCurr = inputFile.pBinary;
        while (bytesLeft > 0)
        {
            uint32_t bytesToWrite = (bytesLeft > 16) ? 16 : bytesLeft;
            uint32_t i;

            fprintf(pOutputFile, "  ");
            for (i = 0 ; i < bytesToWrite ; i++)
            {
                fprintf(pOutputFile, "0x%02X", *pCurr++);
                if (--bytesLeft)
                    fprintf(pOutputFile, ",");
            }
            fprintf(pOutputFile, "\n");
        }

        fprintf(pOutputFile, "};\n");
    }
    __catch
    {
        fprintf(stderr, "error: Encountered unexpected exception %d\n", getExceptionCode());
        returnValue = -1;
    }

    if (pOutputFile)
        fclose(pOutputFile);
    closeBinaryFile(&inputFile);

    return returnValue;
}

static BinaryFile openBinaryFile(const char* binaryFilename)
{
    FILE*      pFile = NULL;
    BinaryFile file;

    pFile = fopen(binaryFilename, "r");
    if (!pFile)
    {
        fprintf(stderr, "error: Failed to open %s: ", binaryFilename);
        perror(NULL);
        __throw(fileException);
    }
    memset(&file, 0, sizeof(file));

    __try
    {
        size_t bytesRead = 0;

        file.binarySize = GetFileSize(pFile);
        file.pBinary = malloc(file.binarySize);
        if (!file.pBinary)
            __throw(outOfMemoryException);
        bytesRead = fread(file.pBinary, 1, file.binarySize, pFile);
        if (bytesRead != file.binarySize)
        {
            fprintf(stderr, "error: Failed to read %u bytes from %s: ", file.binarySize, binaryFilename);
            perror(NULL);
        }
    }
    __catch
    {
        free(file.pBinary);
        fclose(pFile);
        __rethrow;
    }

    fclose(pFile);
    return file;
}

static void closeBinaryFile(BinaryFile* pFile)
{
    if (!pFile)
        return;
    free(pFile->pBinary);
    memset(pFile, 0, sizeof(*pFile));
}

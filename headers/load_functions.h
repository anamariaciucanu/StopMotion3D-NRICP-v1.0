#ifndef LOAD_FUNCTIONS_H
#define LOAD_FUNCTIONS_H

///@file load_functions.h
///@brief loads text from files, used mostly for shaders
///@author Web

#include <stdio.h>
#include <cstdlib>

///@brief loads text from file
///@param [in] _filename -> the file to be opened and read
///@param [out] char* -> string of characters representing the text in the file
char* file_read(const char* _filename)
{
      FILE* input = fopen(_filename, "rb");
      if(input == NULL) return NULL;

      if(fseek(input, 0, SEEK_END) == -1) return NULL;
      long size = ftell(input);
      if(size == -1) return NULL;
      if(fseek(input, 0, SEEK_SET) == -1) return NULL;

      /*if using c-compiler: dont cast malloc's return value*/
      char *content = (char*) malloc( (size_t) size +1 );
      if(content == NULL) return NULL;

      fread(content, 1, (size_t)size, input);
      if(ferror(input)) {
        free(content);
        return NULL;
      }

      fclose(input);
      content[size] = '\0';
      return content;
}


//Aux functions/bits, commented out
/*
    printf("Eigenvectors: \n");
    for(unsigned int i=0; i<3; ++i)
    {
        for(unsigned int j=0; j<3; ++j)
        {
            printf(" %f ", m_eigenvectors(i, j));
        }
        printf("\n");
    }

    printf("rotation matrix: \n");
    for(unsigned int i=0; i<3; ++i)
    {
        for(unsigned int j=0; j<3; ++j)
        {
            printf(" %f ", m_eigenvectors(i, j));
        }
        printf("\n");
    }
*/



#endif // LOAD_FUNCTIONS_H

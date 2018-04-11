/* $Id: debug.h,v 1.1 2018/04/09 12:21:43 ahmet Exp $ */

#ifdef DEBUG
#define PRINTFDB(x) printf x
#else
#define PRINTFDB(x) /* */
#endif

/* sample usage:
   main()
   {
     int i=5,j=6;
     printf("hello1 i=%d\n", i);
     PRINTFDB(("debug i=%d j=%d \n", i, j));
   }
*/

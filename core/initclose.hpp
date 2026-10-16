#ifndef __INIT_CLOSE_H_
#define __INIT_CLOSE_H_

// FILE: SDL/SDL_image startup + shutdown, called once each from main().
// FITS IN: initSDL must run before any Renderer/Texture use; closeDownSDL
// tears down the same singletons. Errors: initSDL logs and exit(1)s on
// failure - it does not return an error code to the caller.
void initSDL(char *windowName);

void closeDownSDL();

#endif // __INIT_CLOSE_H_

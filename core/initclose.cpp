// FILE: SDL/SDL_image lifecycle - see initclose.hpp for the contract.
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <stdio.h>
#include <stdlib.h>
#include "initclose.hpp"
#include "renderer.hpp"
#include "logger.hpp"

// SDL init and close utility functions

void initSDL(char *windowName)
{
    char msg[200];
    if(SDL_Init(SDL_INIT_VIDEO)< 0){
        sprintf(msg,"Could not initialize SDL! SDL Error:%s\n",SDL_GetError());
        Logger::Instance()->writeToLog(msg,"runlog.txt");
        exit(1);
    }

    SDL_SetHint(SDL_HINT_WINDOWS_DISABLE_THREAD_NAMING,"1");

    if(SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY,"0")==SDL_FALSE){

        sprintf(msg,"Warning: Linear texture filtering was not enabled!");
        Logger::Instance()->writeToLog(msg,"runlog.txt");
    }

    Renderer::Instance()->initRenderer(windowName);

    if( !(IMG_Init(IMG_INIT_PNG) & IMG_INIT_PNG))
    {
        sprintf(msg,"Could not initialize SDL_image! SDL_image error:%s\n",IMG_GetError());
        Logger::Instance()->writeToLog(msg,"runlog.txt");
        exit(1);
    }

}

void closeDownSDL()
{
    Renderer::Instance()->closeRenderer();
    IMG_Quit();
    SDL_Quit();
}

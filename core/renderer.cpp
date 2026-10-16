// FILE: Renderer implementation - see renderer.hpp for the contract.
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include "renderer.hpp"
#include "logger.hpp"

Renderer * Renderer::s_pInstance = 0;
SDL_Window * Renderer::window = 0;
SDL_Renderer * Renderer::renderer = 0;

Renderer * Renderer::Instance()
{
	if (s_pInstance == 0)
	{
		s_pInstance = new Renderer();
	}
	return s_pInstance;
}


void Renderer::initRenderer(string windowCaption)
{
    // WATCH OUT: SDL_WINDOW_FULLSCREEN below means the game always launches
    // fullscreen at WINDOW_WIDTH x WINDOW_HEIGHT, regardless of the actual
    // display resolution - not a resizable/windowed mode.
    window = SDL_CreateWindow(windowCaption.c_str(),SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,
                              WINDOW_WIDTH,WINDOW_HEIGHT,SDL_WINDOW_FULLSCREEN);
    if(window == NULL){
        string msg = "SDL_CreateWindow failed:"+string(SDL_GetError());
        Logger::Instance()->writeToLog(msg,"runlog.txt");
        exit(1);
    }

    renderer = SDL_CreateRenderer(window,-1,SDL_RENDERER_ACCELERATED | SDL_RENDERER_TARGETTEXTURE | SDL_RENDERER_PRESENTVSYNC);
    if(renderer == NULL)
    {
        string msg = "SDL_CreateRenderer failed:"+string(SDL_GetError());
        Logger::Instance()->writeToLog(msg,"runlog.txt");
        exit(1);
    }
}

SDL_Renderer *Renderer::getRenderer()
{
    return renderer;
}

SDL_Window *Renderer::getWindow()
{
    return window;
}

void Renderer::closeRenderer()
{
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
}


#include <iostream>
#include <string>
#include "Eigen/Dense" //may only need the dense
#include "Kalman.hpp"
#include "SDL.h"
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h" //TODO: subprojectsin <> or in ""
/*
OPTIONS:
    -a, --accsig <acceleration_sigma>
            Manually specify the accelerometer's error sigma

    -d, --duration <trajectory_duration>
            Specify trajectory duration in minutes from unsigned integer

        --debug
            Enables debug mode which provides true information in addition to the noised information

        --delta
            Print the difference between real position and received position at each reception.

    -e, --entropy
            Generate seed from entropy.

        --filterspeed
            Print the mean of response time from the filter at the end of transmission.

    -g, --gpssig <gps_sigma>
            Manually specify the GPS' error sigma

    -h, --help
            Print help information

    -n, --noise <noise_increase>
            Multiply the noise applied to the accelerometer and GPS

    -p, --port <port>
            Manually specify port for the server. Otherwise defaults to 4242. If the port is closed,
            program defaults to closest open port.

    -s, --seed <manual_seed>
            Generate seed from unsigned integer

    -V, --version
            Print version information
*/

void setup_imgui_context(SDL_Window *sdl_window, SDL_Renderer *sdl_renderer)
{

        ImGui::CreateContext();

        ImGuiIO &io = ImGui::GetIO();
        (void)io;

        // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable keyboard controls
        // io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // Enable Docking
        // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;   // Enable Multi-Viewport / Platform Windows

        // theme
        ImGui::StyleColorsDark();

        // Setup Platform/Renderer backends
        ImGui_ImplSDL2_InitForSDLRenderer(sdl_window, sdl_renderer);
        ImGui_ImplSDLRenderer2_Init(sdl_renderer);

        // Start the Dear ImGui frame
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
}

// void render_loop(SDL_Window *window, SDL_Renderer *renderer, Kalman &kalman_filter)
// {
//         bool running = true;
//         while (running)
//         {
//                 SDL_Event event;
//                 while (SDL_PollEvent(&event))
//                 {
//                         if (event.type == SDL_QUIT)
//                         {
//                                 std::cout << "Exit window event called. Closing the program" << std::endl;
//                                 running = false;
//                         }
//                 }

//                 // Initialize the new frame for ImGui
//                 ImGui::NewFrame();

//                 SDL_SetRenderDrawColor(renderer, 255, 255, 255, 250);
//                 SDL_RenderClear(renderer);

//                 ImGui::Begin("My Window");
//                 ImGui::Text("Color");
//                 ImGui::End();

//                 bool check_kal = kalman_filter.filter_loop();
//                 check_kal = true;
//                 if (!check_kal)
//                 {
//                         ImGui::Begin("Error");
//                         ImGui::Text("Kalman filter has encountered an issue. Pausing...");
//                         ImGui::End();
//                 }

//                 ImGui::Render();
//                 ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());

//                 SDL_RenderPresent(renderer);
//                 if (!check_kal)
//                 {
//                         SDL_Delay(2000);
//                         return;
//                 }
//         }
// }

int main()
{
        bool graphics = true;

        Kalman kalman_filter;

        if (graphics)
        {
                if (SDL_Init(SDL_INIT_VIDEO) < 0)
                {
                        std::cout << "Failed to initialize the SDL2 library\n";
                        return -1;
                }

                SDL_Window *window = SDL_CreateWindow("SDL2 Window",
                                                      SDL_WINDOWPOS_CENTERED,
                                                      SDL_WINDOWPOS_CENTERED,
                                                      680, 480,
                                                      SDL_WINDOW_SHOWN);
                if (!window)
                {
                        std::cout << "Failed to create window\n";
                        return -1;
                }

                SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
                if (!renderer)
                {
                        std::cout << "Failed to create renderer\n";
                        return -1;
                }

                setup_imgui_context(window, renderer);

                kalman_filter.render_loop(window, renderer);

                SDL_DestroyRenderer(renderer);
                SDL_DestroyWindow(window);
                SDL_Quit();
        }
        else
        {
                kalman_filter.filter_loop();
        }

        return 0;
}
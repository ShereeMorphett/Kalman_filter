
#include <iostream>
#include <string>
#include "Eigen/Dense" //may only need the dense
#include "Kalman.hpp"
#include "SDL.h"
#include "imgui.h"
#include "implot.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h" //TODO: subprojectsin <> or in ""
#include "colour.hpp"

#include <mutex>
#include <atomic>
#include <thread>

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

std::mutex get_predictions_mutex;

void setup_imgui_context(SDL_Window *sdl_window, SDL_Renderer *sdl_renderer)
{

        ImGui::CreateContext();
        ImPlot::CreateContext();

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

void render_loop(SDL_Window *window, SDL_Renderer *renderer, Kalman &kalman_filter)
{
        bool running = true;
        float value;
        std::vector<float> x_data;
        std::vector<float> y_data;
        y_data.push_back(0);
        x_data.push_back(y_data.size());

        while (running)
        {
                SDL_Event event;
                while (SDL_PollEvent(&event))
                {
                        if (event.type == SDL_QUIT)
                        {
                                std::cout << "Exit window event called. Closing the program" << std::endl;
                                running = false;
                                return;
                        }
                }

                ImGui::NewFrame();
                SDL_SetRenderDrawColor(renderer, 255, 255, 255, 250);
                SDL_RenderClear(renderer);

                ImGui::SetNextWindowPos(ImVec2(5, 5));
                ImGui::SetNextWindowSize(ImVec2(650, 450));

                get_predictions_mutex.lock();
                value = kalman_filter.get_sent_predictions();
                get_predictions_mutex.unlock();
                if (value != y_data[y_data.size()])
                {
                        y_data.push_back(kalman_filter.get_sent_predictions());
                        x_data.push_back(y_data.size());
                }

                if (kalman_filter.get_kalman_error())
                {
                        std::cout << "Rendering error window" << std::endl;

                        ImGui::Begin("Kalman_filter");

                        ImPlot::SetNextAxesLimits(0, 200, 0, 200, ImPlotCond_Always);

                        if (ImPlot::BeginPlot("Live Plot"))
                        {
                                ImPlot::PlotScatter("My Line Plot", x_data.data(), y_data.data(), y_data.size());
                                ImPlot::EndPlot();
                        }
                        ImGui::End();
                        running = false;
                }
                else
                {

                        ImGui::Begin("Kalman_filter");

                        ImPlot::SetNextAxesLimits(0, 200, 0, 200, ImPlotCond_Always);

                        if (ImPlot::BeginPlot("Live Plot"))
                        {

                                ImPlot::PlotScatter("My Line Plot", x_data.data(), y_data.data(), y_data.size());
                                ImPlot::EndPlot();
                        }

                        ImGui::Text("Kalman filter is fine, plotting live data.");
                        ImGui::End();
                }

                ImGui::Render();
                ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());
                SDL_RenderPresent(renderer);
        }
        SDL_Delay(2000);
        std::cout << COLOR_RED << y_data.size() << COLOR_RESET << std::endl;
}

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

                std::thread filter_thread([&]()
                                          { kalman_filter.filter_loop(); });

                render_loop(window, renderer, kalman_filter);

                filter_thread.join();
                ImGui::DestroyContext();
                ImPlot::DestroyContext();
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
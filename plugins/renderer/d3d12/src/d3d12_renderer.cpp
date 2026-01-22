#include "rkg/host_context.h"
#include "rkg/log.h"
#include "rkg/plugin_api.h"
#include "rkg_platform/platform.h"

#if defined(_WIN32) && defined(RKG_D3D12_SUPPORTED) && RKG_D3D12_SUPPORTED
#include <SDL3/SDL.h>
#include <SDL3/SDL_video.h>
#include <SDL3/SDL_properties.h>

#include <windows.h>
#include <d3d12.h>
#include <d3dcompiler.h>
#include <dxgi1_6.h>
#include <wrl.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using Microsoft::WRL::ComPtr;

namespace {
constexpr uint32_t kFrameCount = 2;
constexpr DXGI_FORMAT kSwapchainFormat = DXGI_FORMAT_R8G8B8A8_UNORM;

struct Vertex {
  float position[3];
  float color[4];
};

struct D3D12State {
  rkg::HostContext* ctx = nullptr;
  SDL_Window* window = nullptr;
  HWND hwnd = nullptr;

  ComPtr<IDXGIFactory4> factory;
  ComPtr<IDXGIAdapter1> adapter;
  ComPtr<ID3D12Device> device;
  ComPtr<ID3D12CommandQueue> command_queue;
  ComPtr<IDXGISwapChain3> swapchain;
  ComPtr<ID3D12DescriptorHeap> rtv_heap;
  UINT rtv_descriptor_size = 0;
  std::array<ComPtr<ID3D12Resource>, kFrameCount> render_targets;

  ComPtr<ID3D12CommandAllocator> command_allocator;
  ComPtr<ID3D12GraphicsCommandList> command_list;
  ComPtr<ID3D12Fence> fence;
  HANDLE fence_event = nullptr;
  UINT64 fence_value = 0;
  UINT frame_index = 0;

  ComPtr<ID3D12RootSignature> root_signature;
  ComPtr<ID3D12PipelineState> pipeline_state;
  ComPtr<ID3D12Resource> vertex_buffer;
  D3D12_VERTEX_BUFFER_VIEW vertex_view{};

  D3D12_VIEWPORT viewport{};
  D3D12_RECT scissor{};
  bool size_valid = true;
  bool initialized = false;
} g_state;

std::string wide_to_utf8(const wchar_t* text) {
  if (!text) return {};
  const int len = static_cast<int>(wcslen(text));
  if (len == 0) return {};
  const int size_needed = WideCharToMultiByte(CP_UTF8, 0, text, len, nullptr, 0, nullptr, nullptr);
  if (size_needed <= 0) return {};
  std::string out(size_needed, '\0');
  WideCharToMultiByte(CP_UTF8, 0, text, len, out.data(), size_needed, nullptr, nullptr);
  return out;
}

HWND get_hwnd(SDL_Window* window) {
  SDL_PropertiesID props = SDL_GetWindowProperties(window);
  return reinterpret_cast<HWND>(SDL_GetPointerProperty(props, SDL_PROP_WINDOW_WIN32_HWND, nullptr));
}

std::string load_shader_source() {
  const char* root = std::getenv("RKG_ROOT");
  std::filesystem::path path = root ? std::filesystem::path(root) : std::filesystem::current_path();
  path /= "plugins/renderer/d3d12/shaders/triangle.hlsl";
  std::ifstream in(path);
  if (in) {
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  }
  return R"(
struct VSInput {
  float3 position : POSITION;
  float4 color : COLOR0;
};

struct PSInput {
  float4 position : SV_POSITION;
  float4 color : COLOR0;
};

PSInput VSMain(VSInput input) {
  PSInput output;
  output.position = float4(input.position, 1.0f);
  output.color = input.color;
  return output;
}

float4 PSMain(PSInput input) : SV_TARGET {
  return input.color;
}
)";
}

bool compile_shader(const std::string& source,
                    const char* entry,
                    const char* target,
                    ComPtr<ID3DBlob>& out_blob) {
  UINT flags = D3DCOMPILE_ENABLE_STRICTNESS;
#if defined(_DEBUG)
  flags |= D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;
#endif
  ComPtr<ID3DBlob> errors;
  HRESULT hr = D3DCompile(source.data(),
                          source.size(),
                          nullptr,
                          nullptr,
                          nullptr,
                          entry,
                          target,
                          flags,
                          0,
                          &out_blob,
                          &errors);
  if (FAILED(hr)) {
    if (errors) {
      const char* msg = static_cast<const char*>(errors->GetBufferPointer());
      if (msg) {
        rkg::log::error(std::string("d3d12 shader compile error: ") + msg);
      }
    }
    return false;
  }
  return true;
}

bool create_device() {
  HRESULT hr = CreateDXGIFactory1(IID_PPV_ARGS(&g_state.factory));
  if (FAILED(hr)) {
    rkg::log::error("d3d12: CreateDXGIFactory1 failed");
    return false;
  }

  for (UINT i = 0; ; ++i) {
    ComPtr<IDXGIAdapter1> adapter;
    if (g_state.factory->EnumAdapters1(i, &adapter) == DXGI_ERROR_NOT_FOUND) {
      break;
    }
    DXGI_ADAPTER_DESC1 desc{};
    adapter->GetDesc1(&desc);
    if (desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) {
      continue;
    }
    if (SUCCEEDED(D3D12CreateDevice(adapter.Get(), D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&g_state.device)))) {
      g_state.adapter = adapter;
      rkg::log::info(std::string("d3d12 adapter: ") + wide_to_utf8(desc.Description));
      return true;
    }
  }

  ComPtr<IDXGIAdapter> warp;
  if (SUCCEEDED(g_state.factory->EnumWarpAdapter(IID_PPV_ARGS(&warp))) &&
      SUCCEEDED(D3D12CreateDevice(warp.Get(), D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&g_state.device)))) {
    rkg::log::warn("d3d12: using WARP adapter");
    return true;
  }

  rkg::log::error("d3d12: no suitable adapter found");
  return false;
}

bool create_command_queue() {
  D3D12_COMMAND_QUEUE_DESC desc{};
  desc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
  desc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
  if (FAILED(g_state.device->CreateCommandQueue(&desc, IID_PPV_ARGS(&g_state.command_queue)))) {
    rkg::log::error("d3d12: CreateCommandQueue failed");
    return false;
  }
  return true;
}

bool create_swapchain() {
  int width = 1280;
  int height = 720;
  SDL_GetWindowSize(g_state.window, &width, &height);

  DXGI_SWAP_CHAIN_DESC1 desc{};
  desc.Width = static_cast<UINT>(width);
  desc.Height = static_cast<UINT>(height);
  desc.Format = kSwapchainFormat;
  desc.BufferCount = kFrameCount;
  desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
  desc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
  desc.SampleDesc.Count = 1;

  ComPtr<IDXGISwapChain1> swapchain1;
  HRESULT hr = g_state.factory->CreateSwapChainForHwnd(
      g_state.command_queue.Get(), g_state.hwnd, &desc, nullptr, nullptr, &swapchain1);
  if (FAILED(hr)) {
    rkg::log::error("d3d12: CreateSwapChainForHwnd failed");
    return false;
  }

  g_state.factory->MakeWindowAssociation(g_state.hwnd, DXGI_MWA_NO_ALT_ENTER);
  if (FAILED(swapchain1.As(&g_state.swapchain))) {
    rkg::log::error("d3d12: swapchain cast failed");
    return false;
  }
  g_state.frame_index = g_state.swapchain->GetCurrentBackBufferIndex();
  rkg::log::info("d3d12: swapchain format DXGI_FORMAT_R8G8B8A8_UNORM");
  return true;
}

bool create_rtv_heap() {
  D3D12_DESCRIPTOR_HEAP_DESC desc{};
  desc.NumDescriptors = kFrameCount;
  desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
  desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
  if (FAILED(g_state.device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&g_state.rtv_heap)))) {
    rkg::log::error("d3d12: CreateDescriptorHeap failed");
    return false;
  }
  g_state.rtv_descriptor_size = g_state.device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
  return true;
}

bool create_render_targets() {
  D3D12_CPU_DESCRIPTOR_HANDLE handle = g_state.rtv_heap->GetCPUDescriptorHandleForHeapStart();
  for (UINT i = 0; i < kFrameCount; ++i) {
    if (FAILED(g_state.swapchain->GetBuffer(i, IID_PPV_ARGS(&g_state.render_targets[i])))) {
      rkg::log::error("d3d12: swapchain GetBuffer failed");
      return false;
    }
    g_state.device->CreateRenderTargetView(g_state.render_targets[i].Get(), nullptr, handle);
    handle.ptr += static_cast<SIZE_T>(g_state.rtv_descriptor_size);
  }
  return true;
}

bool create_command_list() {
  if (FAILED(g_state.device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT,
                                                   IID_PPV_ARGS(&g_state.command_allocator)))) {
    rkg::log::error("d3d12: CreateCommandAllocator failed");
    return false;
  }
  if (FAILED(g_state.device->CreateCommandList(0,
                                               D3D12_COMMAND_LIST_TYPE_DIRECT,
                                               g_state.command_allocator.Get(),
                                               nullptr,
                                               IID_PPV_ARGS(&g_state.command_list)))) {
    rkg::log::error("d3d12: CreateCommandList failed");
    return false;
  }
  g_state.command_list->Close();
  return true;
}

bool create_fence() {
  if (FAILED(g_state.device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g_state.fence)))) {
    rkg::log::error("d3d12: CreateFence failed");
    return false;
  }
  g_state.fence_value = 1;
  g_state.fence_event = CreateEvent(nullptr, FALSE, FALSE, nullptr);
  if (!g_state.fence_event) {
    rkg::log::error("d3d12: CreateEvent failed");
    return false;
  }
  return true;
}

bool create_pipeline() {
  const auto source = load_shader_source();
  ComPtr<ID3DBlob> vs;
  ComPtr<ID3DBlob> ps;
  if (!compile_shader(source, "VSMain", "vs_5_0", vs)) return false;
  if (!compile_shader(source, "PSMain", "ps_5_0", ps)) return false;

  D3D12_ROOT_SIGNATURE_DESC root_desc{};
  root_desc.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;
  ComPtr<ID3DBlob> serialized;
  ComPtr<ID3DBlob> errors;
  if (FAILED(D3D12SerializeRootSignature(&root_desc, D3D_ROOT_SIGNATURE_VERSION_1, &serialized, &errors))) {
    if (errors) {
      const char* msg = static_cast<const char*>(errors->GetBufferPointer());
      if (msg) {
        rkg::log::error(std::string("d3d12 root signature error: ") + msg);
      }
    } else {
      rkg::log::error("d3d12: root signature serialization failed");
    }
    return false;
  }
  if (FAILED(g_state.device->CreateRootSignature(0,
                                                 serialized->GetBufferPointer(),
                                                 serialized->GetBufferSize(),
                                                 IID_PPV_ARGS(&g_state.root_signature)))) {
    rkg::log::error("d3d12: CreateRootSignature failed");
    return false;
  }

  D3D12_INPUT_ELEMENT_DESC layout[] = {
      {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
      {"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
  };

  D3D12_RASTERIZER_DESC raster{};
  raster.FillMode = D3D12_FILL_MODE_SOLID;
  raster.CullMode = D3D12_CULL_MODE_BACK;
  raster.FrontCounterClockwise = FALSE;
  raster.DepthClipEnable = TRUE;

  D3D12_BLEND_DESC blend{};
  blend.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;

  D3D12_DEPTH_STENCIL_DESC depth{};
  depth.DepthEnable = FALSE;
  depth.StencilEnable = FALSE;

  D3D12_GRAPHICS_PIPELINE_STATE_DESC pso{};
  pso.InputLayout = {layout, static_cast<UINT>(std::size(layout))};
  pso.pRootSignature = g_state.root_signature.Get();
  pso.VS = {vs->GetBufferPointer(), vs->GetBufferSize()};
  pso.PS = {ps->GetBufferPointer(), ps->GetBufferSize()};
  pso.RasterizerState = raster;
  pso.BlendState = blend;
  pso.DepthStencilState = depth;
  pso.SampleMask = UINT_MAX;
  pso.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
  pso.NumRenderTargets = 1;
  pso.RTVFormats[0] = kSwapchainFormat;
  pso.SampleDesc.Count = 1;

  if (FAILED(g_state.device->CreateGraphicsPipelineState(&pso, IID_PPV_ARGS(&g_state.pipeline_state)))) {
    rkg::log::error("d3d12: CreateGraphicsPipelineState failed");
    return false;
  }
  return true;
}

bool create_vertex_buffer() {
  const std::array<Vertex, 3> vertices = {{
      {{0.0f, 0.5f, 0.0f}, {0.15f, 0.65f, 0.95f, 1.0f}},
      {{0.5f, -0.5f, 0.0f}, {0.15f, 0.65f, 0.95f, 1.0f}},
      {{-0.5f, -0.5f, 0.0f}, {0.15f, 0.65f, 0.95f, 1.0f}},
  }};

  const UINT buffer_size = static_cast<UINT>(sizeof(vertices));
  D3D12_HEAP_PROPERTIES heap{};
  heap.Type = D3D12_HEAP_TYPE_UPLOAD;
  heap.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
  heap.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;

  D3D12_RESOURCE_DESC desc{};
  desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
  desc.Width = buffer_size;
  desc.Height = 1;
  desc.DepthOrArraySize = 1;
  desc.MipLevels = 1;
  desc.SampleDesc.Count = 1;
  desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

  if (FAILED(g_state.device->CreateCommittedResource(&heap,
                                                     D3D12_HEAP_FLAG_NONE,
                                                     &desc,
                                                     D3D12_RESOURCE_STATE_GENERIC_READ,
                                                     nullptr,
                                                     IID_PPV_ARGS(&g_state.vertex_buffer)))) {
    rkg::log::error("d3d12: CreateCommittedResource failed");
    return false;
  }

  void* mapped = nullptr;
  D3D12_RANGE range{0, 0};
  g_state.vertex_buffer->Map(0, &range, &mapped);
  std::memcpy(mapped, vertices.data(), buffer_size);
  g_state.vertex_buffer->Unmap(0, nullptr);

  g_state.vertex_view.BufferLocation = g_state.vertex_buffer->GetGPUVirtualAddress();
  g_state.vertex_view.SizeInBytes = buffer_size;
  g_state.vertex_view.StrideInBytes = sizeof(Vertex);
  return true;
}

void wait_for_gpu() {
  const UINT64 fence_to_wait = g_state.fence_value;
  g_state.command_queue->Signal(g_state.fence.Get(), fence_to_wait);
  g_state.fence_value++;
  if (g_state.fence->GetCompletedValue() < fence_to_wait) {
    g_state.fence->SetEventOnCompletion(fence_to_wait, g_state.fence_event);
    WaitForSingleObject(g_state.fence_event, INFINITE);
  }
}

bool record_and_submit() {
  if (FAILED(g_state.command_allocator->Reset())) return false;
  if (FAILED(g_state.command_list->Reset(g_state.command_allocator.Get(), g_state.pipeline_state.Get()))) return false;

  g_state.command_list->SetGraphicsRootSignature(g_state.root_signature.Get());
  g_state.command_list->RSSetViewports(1, &g_state.viewport);
  g_state.command_list->RSSetScissorRects(1, &g_state.scissor);

  D3D12_RESOURCE_BARRIER to_render{};
  to_render.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  to_render.Transition.pResource = g_state.render_targets[g_state.frame_index].Get();
  to_render.Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
  to_render.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
  to_render.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
  g_state.command_list->ResourceBarrier(1, &to_render);

  D3D12_CPU_DESCRIPTOR_HANDLE rtv_handle = g_state.rtv_heap->GetCPUDescriptorHandleForHeapStart();
  rtv_handle.ptr += static_cast<SIZE_T>(g_state.frame_index) *
                    static_cast<SIZE_T>(g_state.rtv_descriptor_size);
  g_state.command_list->OMSetRenderTargets(1, &rtv_handle, FALSE, nullptr);

  const float clear_color[] = {0.02f, 0.02f, 0.05f, 1.0f};
  g_state.command_list->ClearRenderTargetView(rtv_handle, clear_color, 0, nullptr);

  g_state.command_list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
  g_state.command_list->IASetVertexBuffers(0, 1, &g_state.vertex_view);
  g_state.command_list->DrawInstanced(3, 1, 0, 0);

  D3D12_RESOURCE_BARRIER to_present{};
  to_present.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  to_present.Transition.pResource = g_state.render_targets[g_state.frame_index].Get();
  to_present.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
  to_present.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
  to_present.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
  g_state.command_list->ResourceBarrier(1, &to_present);

  if (FAILED(g_state.command_list->Close())) return false;

  ID3D12CommandList* lists[] = {g_state.command_list.Get()};
  g_state.command_queue->ExecuteCommandLists(1, lists);

  if (FAILED(g_state.swapchain->Present(1, 0))) {
    return false;
  }

  wait_for_gpu();
  g_state.frame_index = g_state.swapchain->GetCurrentBackBufferIndex();
  return true;
}

bool resize_swapchain(UINT width, UINT height) {
  if (!g_state.initialized) return false;
  if (width == 0 || height == 0) {
    g_state.size_valid = false;
    g_state.viewport = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    g_state.scissor = {0, 0, 0, 0};
    return true;
  }

  rkg::log::info("d3d12: resize begin " + std::to_string(width) + "x" + std::to_string(height));
  wait_for_gpu();

  for (auto& rt : g_state.render_targets) {
    rt.Reset();
  }

  HRESULT hr = g_state.swapchain->ResizeBuffers(kFrameCount,
                                                width,
                                                height,
                                                kSwapchainFormat,
                                                0);
  if (FAILED(hr)) {
    std::ostringstream oss;
    oss << "d3d12: ResizeBuffers failed (0x" << std::hex << static_cast<uint32_t>(hr) << ")";
    rkg::log::error(oss.str());
    return false;
  }

  if (!create_render_targets()) {
    rkg::log::error("d3d12: RTV rebuild failed");
    return false;
  }

  g_state.frame_index = g_state.swapchain->GetCurrentBackBufferIndex();
  g_state.viewport = {0.0f, 0.0f, static_cast<float>(width), static_cast<float>(height), 0.0f, 1.0f};
  g_state.scissor = {0, 0, static_cast<LONG>(width), static_cast<LONG>(height)};
  g_state.size_valid = true;
  rkg::log::info("d3d12: resize complete");
  return true;
}

bool d3d12_init(void* host) {
  g_state.ctx = static_cast<rkg::HostContext*>(host);
  if (!g_state.ctx || !g_state.ctx->platform) {
    rkg::log::error("d3d12: host context missing platform");
    return false;
  }

  g_state.window = static_cast<SDL_Window*>(g_state.ctx->platform->native_window());
  if (!g_state.window) {
    rkg::log::error("d3d12: SDL window missing");
    return false;
  }

  g_state.hwnd = get_hwnd(g_state.window);
  if (!g_state.hwnd) {
    rkg::log::error("d3d12: HWND not available from SDL");
    return false;
  }

  if (!create_device()) return false;
  if (!create_command_queue()) return false;
  if (!create_swapchain()) return false;
  if (!create_rtv_heap()) return false;
  if (!create_render_targets()) return false;
  if (!create_command_list()) return false;
  if (!create_fence()) return false;
  if (!create_pipeline()) return false;
  if (!create_vertex_buffer()) return false;

  int width = 1280;
  int height = 720;
  SDL_GetWindowSize(g_state.window, &width, &height);
  g_state.viewport = {0.0f, 0.0f, static_cast<float>(width), static_cast<float>(height), 0.0f, 1.0f};
  g_state.scissor = {0, 0, width, height};
  g_state.size_valid = (width > 0 && height > 0);

  g_state.initialized = true;
  rkg::log::info("renderer:d3d12 init");
  return true;
}

void d3d12_on_window_resized(int width, int height) {
  if (!g_state.initialized) return;
  if (width <= 0 || height <= 0) {
    rkg::log::info("d3d12: window minimized");
    g_state.size_valid = false;
    return;
  }
  resize_swapchain(static_cast<UINT>(width), static_cast<UINT>(height));
}

void d3d12_shutdown() {
  if (g_state.initialized && g_state.command_queue && g_state.fence) {
    wait_for_gpu();
  }
  if (g_state.fence_event) {
    CloseHandle(g_state.fence_event);
    g_state.fence_event = nullptr;
  }
  g_state = D3D12State{};
  rkg::log::info("renderer:d3d12 shutdown");
}

void d3d12_update(float) {
  if (!g_state.initialized || !g_state.size_valid) return;
  if (!record_and_submit()) {
    rkg::log::warn("renderer:d3d12 draw failed");
  }
}

#else

bool d3d12_init(void* host) {
  (void)host;
  rkg::log::warn("renderer:d3d12 not supported on this platform");
  return false;
}

void d3d12_shutdown() {
  rkg::log::info("renderer:d3d12 shutdown");
}

void d3d12_update(float) {}

#endif

namespace {
rkg::RkgPluginApi g_api{
    rkg::kRkgPluginApiVersion,
    "renderer_d3d12",
    rkg::PluginType::Renderer,
    static_cast<uint32_t>(rkg::RendererCaps::DrawPrimitive),
    &d3d12_init,
    &d3d12_shutdown,
    &d3d12_update,
    &d3d12_on_window_resized};
} // namespace

extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_renderer_d3d12(uint32_t host_api_version) {
  if (host_api_version != rkg::kRkgPluginApiVersion) {
    return nullptr;
  }
  return &g_api;
}

#if defined(RKG_BUILD_DYNAMIC_PLUGIN)
extern "C" rkg::RkgPluginApi* rkg_plugin_get_api(uint32_t host_api_version) {
  return rkg_plugin_get_api_renderer_d3d12(host_api_version);
}
#endif

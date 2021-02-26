#include "Utils.h"

#include <stb/igl_stb_image.h>
#include <algorithm>

namespace utils {

void CaptureScreen(igl::opengl::ViewerCore& core,
                   std::vector<igl::opengl::ViewerData>& data_list,
                   unsigned width,
                   unsigned height,
                   const std::string& filename) {
  bool update_matrices = true;

  // clang-format off
  // https://learnopengl.com/Advanced-OpenGL/Anti-Aliasing
  unsigned int framebuffer;
  glGenFramebuffers(1, &framebuffer);
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
  // create a multisampled color attachment texture
  unsigned int textureColorBufferMultiSampled;
  glGenTextures(1, &textureColorBufferMultiSampled);
  glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, textureColorBufferMultiSampled);
  glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGBA, width, height, GL_TRUE);
  glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, textureColorBufferMultiSampled, 0);
  // create a (also multisampled) renderbuffer object for depth and stencil attachments
  unsigned int rbo;
  glGenRenderbuffers(1, &rbo);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH24_STENCIL8, width, height);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);
  assert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // configure second post-processing framebuffer
  unsigned int intermediateFBO;
  glGenFramebuffers(1, &intermediateFBO);
  glBindFramebuffer(GL_FRAMEBUFFER, intermediateFBO);
  // create a color attachment texture
  unsigned int screenTexture;
  glGenTextures(1, &screenTexture);
  glBindTexture(GL_TEXTURE_2D, screenTexture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, screenTexture, 0);	// we only need a color buffer
  assert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

  // Clear the buffer
  glClearColor(core.background_color(0), core.background_color(1), core.background_color(2), 0.f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  // Save old viewport
  Eigen::Vector4f viewport_ori = core.viewport;
  core.viewport << 0,0,width,height;
  // Draw
  for (auto& mesh : data_list) {
    if (mesh.is_visible) {
      core.draw(mesh, update_matrices);
    }
  }
  // Restore viewport
  core.viewport = viewport_ori;

  glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, intermediateFBO);
  glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST);

  glBindFramebuffer(GL_FRAMEBUFFER, intermediateFBO);
  // Copy back in the given Eigen matrices
  GLubyte* pixels = (GLubyte*)calloc(width*height*4,sizeof(GLubyte));
  glReadPixels(0, 0,width, height,GL_RGBA, GL_UNSIGNED_BYTE, pixels);

  // Clean up
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
  glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glDeleteTextures(1, &screenTexture);
  glDeleteTextures(1, &textureColorBufferMultiSampled);
  glDeleteFramebuffers(1, &framebuffer);
  glDeleteFramebuffers(1, &intermediateFBO);
  glDeleteRenderbuffers(1, &rbo);
  // clang-format on

  // flip image
  std::reverse(pixels, pixels + (width * height * 4));
  for (unsigned i = 0; i < height; i++) {
    std::reverse(pixels + (i * width * 4), pixels + ((i + 1) * width * 4));
  }

  igl::stbi_write_png(filename.c_str(), width, height, 4, pixels, width * 4);

  // Clean up
  free(pixels);
}

}  // namespace utils
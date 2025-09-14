set(LA9310_SOURCES
  LA9310.cpp
  VSPA_mailbox.cpp
  VSPA_iqplayer.cpp
)

target_sources(limesuiteng PRIVATE ${LA9310_SOURCES})

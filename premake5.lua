workspace "iron-sight"
    configurations {"Debug", "Release"}

project "iron-sight"
    kind "ConsoleApp"
    language "C++"
    targetdir "bin/%{cfg.buildcfg}"

    files { "**.h", "**.cc" }

    filter "configurations:Release"
        defines { "NDEBUG" }
        optimize "On"
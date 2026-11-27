import shutil, sys, time
PATH = "src/optical_flow_mavlink.cpp"
src = open(PATH, encoding="utf-8").read()
E = []
E.append(("""        // VIRTUAL_RANGEFINDER_SHADOW_V1
        // Diagnostic only. Never feeds production or MAVLink.
        {
""","""        // BARO_HEIGHT_V1: height chosen by the VRF block (RC6 BARO / AUTO failover)
        // replaces TF-Luna in the FC range message and in camera height.
        double vrf_height_m=virtual_rangefinder::invalidHeight();
        bool vrf_height_baro=false;

        // VIRTUAL_RANGEFINDER_SHADOW_V1
        {
"""))
E.append(("""          const char* mode_name=
""","""          vrf_height_m=selected_m;
          vrf_height_baro=selected_valid && vrf_source==2;

          const char* mode_name=
"""))
E.append(("""        const bool range_available =
          bench_height_override>0.0 || have_last_valid_luna;

        const double published_range_m =
          bench_height_override>0.0
            ? bench_height_override
            : last_valid_luna_m;
""","""        const bool use_baro_h =
          bench_height_override<=0.0 && vrf_height_baro &&
          std::isfinite(vrf_height_m) && vrf_height_m>=0.03;

        const bool range_available =
          bench_height_override>0.0 || have_last_valid_luna || use_baro_h;

        const double published_range_m =
          bench_height_override>0.0
            ? bench_height_override
            : (use_baro_h ? vrf_height_m : last_valid_luna_m);
"""))
E.append(("""        double current_camera_height_m=0.0;
        bool current_camera_height_valid=hl && lage<100.0 && lm>=0.03;
        if(current_camera_height_valid){
          current_camera_height_m=lm;
""","""        const double h_src_m = use_baro_h ? vrf_height_m : lm;
        double current_camera_height_m=0.0;
        bool current_camera_height_valid=use_baro_h || (hl && lage<100.0 && lm>=0.03);
        if(current_camera_height_valid){
          current_camera_height_m=h_src_m;
"""))
E.append(("""            current_camera_height_m=lm-(diag_camera_z_m-diag_range_z_m);""",
"""            current_camera_height_m=h_src_m-(diag_camera_z_m-diag_range_z_m);"""))
for i,(o,n) in enumerate(E,1):
    c=src.count(o)
    if c!=1:
        print(f"ОШИБКА: фрагмент {i} найден {c} раз, нужно 1. Файл не изменён."); sys.exit(1)
for o,n in E: src=src.replace(o,n,1)
bak=PATH+".before_baro_height_"+time.strftime("%Y%m%d_%H%M%S")
shutil.copy2(PATH,bak); open(PATH,"w",encoding="utf-8").write(src)
print("Готово. Бэкап:",bak,"| правок:",len(E))

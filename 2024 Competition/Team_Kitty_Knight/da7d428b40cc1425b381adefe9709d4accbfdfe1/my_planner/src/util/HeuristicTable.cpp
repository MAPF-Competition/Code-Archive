#include "util/HeuristicTable.h"

namespace UTIL {

HeuristicTable::HeuristicTable(SharedEnvironment * _env, const std::shared_ptr<std::vector<float> > & map_weights, bool consider_rotation):
    env(*_env),
    consider_rotation(consider_rotation),
    map_weights(map_weights) {
    if (consider_rotation) {
        n_orientations=4;
    } else {
        n_orientations=1;
    }

    loc_size=0;
    for (int i=0;i<env.map.size();++i) {
        if (!env.map[i]) {
            ++loc_size;
        }
    }

    empty_locs = new int[loc_size];
    loc_idxs = new int[env.map.size()];

    ONLYDEV(std::cout<<"number of empty locations: "<<loc_size<<std::endl;);
    std::fill(loc_idxs,loc_idxs+env.map.size(),-1);

    int loc_idx=0;
    for (int loc=0;loc<env.map.size();++loc) {
        if (!env.map[loc]) {
            empty_locs[loc_idx]=loc;
            loc_idxs[loc]=loc_idx;
            ++loc_idx;
        }
    }
    ONLYDEV(assert(loc_idx==loc_size);)

    state_size = loc_size*n_orientations;
    main_heuristics = new float[loc_size*loc_size];
    std::fill(main_heuristics,main_heuristics+loc_size*loc_size,MAX_HEURISTIC);
    // we keep start_loc, end_loc, start_orient, namely no goal_orient
    if (consider_rotation)
        sub_heuristics = new float[state_size*loc_size];
};

HeuristicTable::~HeuristicTable() {
    delete [] empty_locs;
    delete [] loc_idxs;
    delete [] main_heuristics;
    if (consider_rotation)
        delete [] sub_heuristics;
}

// weights is an array of [loc_size*n_orientations]
void HeuristicTable::compute_weighted_heuristics(){
    ONLYDEV(std::cout<<"[start] Compute heuristics."<<std::endl;);
    ONLYDEV(g_timer.record_p("heu/compute_start");)

    int n_threads=omp_get_max_threads();
    // BS::thread_pool pool(n_threads);

    // int n_threads=pool.get_thread_count();
    cout<<"number of threads used for heuristic computation: "<<n_threads<<endl;
    float * values = new float[n_threads*n_orientations*state_size];
    UTIL::SPATIAL::SpatialAStar ** planners= new UTIL::SPATIAL::SpatialAStar* [n_threads];
    for (int i=0;i<n_threads;++i) {
        planners[i]=new UTIL::SPATIAL::SpatialAStar(env,n_orientations,*map_weights);
    }

    cerr<<"created"<<endl;

    int ctr=0;
    int step=100;
    auto start = std::chrono::steady_clock::now();

    #pragma omp parallel for schedule(dynamic,1)
    for (int loc_idx=0;loc_idx<loc_size;++loc_idx)
    {
        int thread_id=omp_get_thread_num();

        int s_idx=thread_id*n_orientations*state_size;

        _compute_weighted_heuristics(loc_idx,values+s_idx,planners[thread_id]);


        #pragma omp critical
        {
            ++ctr;
            if (ctr%step==0){
                auto end = std::chrono::steady_clock::now();
                double elapse=std::chrono::duration<double>(end-start).count();
                double estimated_remain=elapse/ctr*(loc_size-ctr);
                cout<<ctr<<"/"<<loc_size<<" completed in "<<elapse<<"s. estimated time to finish all: "<<estimated_remain<<"s.  estimated total time: "<<(estimated_remain+elapse)<<"s."<<endl;
            }
        }

        // ONLYDEV(if (empty_locs[loc_idx]==2) {
        //     dump_main_heuristics(
        //         2,
        //         "analysis/heuristics/debug"
        //     );
        // })
    }

    delete [] values;
    for (int i=0;i<n_threads;++i) {
        delete planners[i];
    }
    delete [] planners;

    ONLYDEV(g_timer.record_d("heu/compute_start","heu/compute_end","heu/compute");)

    ONLYDEV(std::cout<<"[end] Compute heuristics. (duration: )"<<g_timer.get_d("heu/compute")<<std::endl;);
}

void HeuristicTable::_compute_weighted_heuristics(
    int start_loc_idx,
    float * values,
    UTIL::SPATIAL::SpatialAStar * planner
) {
    int start_loc=empty_locs[start_loc_idx];
    if (!consider_rotation){
        planner->reset();
        planner->search_for_all(start_loc,-1);
        
        for (int loc_idx=0;loc_idx<loc_size;++loc_idx) {
            int loc=empty_locs[loc_idx];
            UTIL::SPATIAL::State * state=planner->all_states+loc;
            if (state->pos!=-1 && loc!=state->pos) {
                std::cerr<<"loc: "<<loc<<" state->pos: "<<state->pos<<endl;
                exit(-1);
            }
            float cost=state->g;
            if (cost==-1) {
                cost=MAX_HEURISTIC;
            }
            size_t main_idx=start_loc_idx*loc_size+loc_idx;
            main_heuristics[main_idx]=cost;
        }
    } else {
        std::fill(values,values+n_orientations*state_size,MAX_HEURISTIC);
        for (int start_orient=0;start_orient<n_orientations;++start_orient){
            planner->reset();
            planner->search_for_all(start_loc,start_orient);
            for (int loc_idx=0;loc_idx<loc_size;++loc_idx) {
                for (int orient=0;orient<n_orientations;++orient) {
                    int loc=empty_locs[loc_idx];
                    UTIL::SPATIAL::State * state=planner->all_states+loc*n_orientations+orient;
                    // if (loc!=state->pos || orient!=state->orient) {
                    //     std::cerr<<"start_loc: "<<start_loc<<" loc: "<<loc<<" state->pos: "<<state->pos<<" orient: "<<orient<<" state->orient: "<<state->orient<<endl;
                    //     exit(-1);
                    // }
                    float cost=state->g;
                    if (cost==-1) {
                        cost=MAX_HEURISTIC;
                    }
                    if (cost>MAX_HEURISTIC) {
                        std::cerr<<"cost: "<<cost<<" > "<<MAX_HEURISTIC<<endl;
                        exit(-1);
                    }

                    size_t value_idx=start_orient*state_size+loc_idx*n_orientations+orient;
                    values[value_idx]=cost;

                    size_t main_idx=start_loc_idx*loc_size+loc_idx;
                    if (cost<main_heuristics[main_idx]){
                        main_heuristics[main_idx]=cost;
                    }
                }
            }
        }

        for (int start_orient=0;start_orient<n_orientations;++start_orient){
            for (int loc_idx=0;loc_idx<loc_size;++loc_idx) {
                float cost=MAX_HEURISTIC;
                for (int orient=0;orient<n_orientations;++orient) {
                    size_t value_idx=start_orient*state_size+loc_idx*n_orientations+orient;
                    auto value=values[value_idx];
                    if (value<cost) {
                        cost=value;
                    }
                }
                size_t sub_idx=(start_loc_idx*loc_size+loc_idx)*n_orientations+start_orient;
                size_t main_idx=start_loc_idx*loc_size+loc_idx;
                float diff=cost-main_heuristics[main_idx];
                if (diff<0) {
                    std::cerr<<"diff: "<<diff<<" < 0"<<endl;
                    exit(-1);
                }

                if (diff>MAX_HEURISTIC) {
                    std::cerr<<"diff: "<<diff<<" > "<<MAX_HEURISTIC<<endl;
                    exit(-1);
                }
                sub_heuristics[sub_idx]=diff;
            }
        }
    }
}

void HeuristicTable::dump_main_heuristics(int start_loc, string file_path_prefix) {
    int start_loc_idx=loc_idxs[start_loc];
    if (start_loc_idx==-1) {
        std::cerr<<"error: start_loc_idx==-1"<<endl;
        exit(-1);
    }
    string file_path=file_path_prefix+"_"+std::to_string(start_loc)+".main_heuristics";
    std::ofstream fout(file_path);
    for (int i=0;i<env.rows;++i) {
        for (int j=0;j<env.cols;++j) {
            int target_loc=i*env.cols+j;
            int target_loc_idx=loc_idxs[target_loc];
            float h=MAX_HEURISTIC;
            if (target_loc_idx!=-1){
                h=main_heuristics[start_loc_idx*loc_size+target_loc_idx];
            }
            fout<<h;
            if (j!=env.cols-1) {
                fout<<",";
            }
        }
        fout<<endl;
    }
}


float HeuristicTable::get_edge_weight(int from, int to) {
    int diff=to-from;
    if (diff==1) { // move right
        return (*map_weights)[from*5];
    } else if (diff==env.cols) {
        return (*map_weights)[from*5+1];
    } else if (diff==-1) {
        return (*map_weights)[from*5+2];
    } else if (diff==-env.cols) {
        return (*map_weights)[from*5+3];
    } else if (diff==0) {
        return (*map_weights)[from*5+4];
    } else {
        std::cerr<<"error: from: "<<from<<" to: "<<to<<endl;
        exit(-1);
    }
}

// TODO add check
float HeuristicTable::get(int loc1, int loc2) {
    int loc_idx1=loc_idxs[loc1];
    int loc_idx2=loc_idxs[loc2];

    if (loc_idx1==-1 || loc_idx2==-1) {
        return MAX_HEURISTIC;
    }

    size_t idx=loc_idx1*loc_size+loc_idx2;
    return main_heuristics[idx];
}

float HeuristicTable::get(int loc1, int orient1, int loc2) {
    if (!consider_rotation) {
        std::cerr<<"no valid to use this func if not consider rotation"<<endl;
        exit(-1);
    }

    int loc_idx1=loc_idxs[loc1];
    int loc_idx2=loc_idxs[loc2];

    if (loc_idx1==-1 || loc_idx2==-1) {
        return MAX_HEURISTIC;
    }

    size_t main_idx=loc_idx1*loc_size+loc_idx2;
    size_t sub_idx=(loc_idx1*loc_size+loc_idx2)*n_orientations+orient1;
    return main_heuristics[main_idx]+sub_heuristics[sub_idx];
} 

void HeuristicTable::preprocess(string suffix) {

    string fname=env.map_name.substr(0,env.map_name.size()-4);
    string folder=env.file_storage_path;
    if (folder[folder.size()-1]!=boost::filesystem::path::preferred_separator){
        folder+=boost::filesystem::path::preferred_separator;
    }
    string fpath;
    if (consider_rotation) {
        fpath=folder+fname+"_weighted_heuristics_v4_"+suffix+".gz";
    } else {
        fpath=folder+fname+"_weighted_heuristics_no_rotation_v4_"+suffix+".gz";
    }

    if (boost::filesystem::exists(fpath)) {
        load(fpath);
    } else {
        compute_weighted_heuristics();
    }
}

void HeuristicTable::save(const string & fpath) {
    ONLYDEV(std::cout<<"[start] Save heuristics to "<<fpath<<std::endl;);
    ONLYDEV(g_timer.record_p("heu/save_start");)

    std::ofstream fout;
    fout.open(fpath,std::ios::binary|std::ios::out);

    boost::iostreams::filtering_streambuf<boost::iostreams::output> outbuf;
    outbuf.push(boost::iostreams::zlib_compressor());
    outbuf.push(fout);
    
    std::ostream out(&outbuf);

    // save loc size
    out.write((char *)&loc_size,sizeof(int));

    // save empty locs
    out.write((char*)empty_locs,sizeof(int)*loc_size);

    // save main heuristics
    out.write((char *)main_heuristics,sizeof(float)*loc_size*loc_size);

    // save sub heuristics
    if (consider_rotation)
        out.write((char *)sub_heuristics,sizeof(float)*state_size*loc_size);

    boost::iostreams::close(outbuf);
    fout.close();
    
    ONLYDEV(g_timer.record_d("heu/save_start","heu/save_end","heu/save");)

    ONLYDEV(std::cout<<"[end] Save heuristics to "<<fpath<<". (duration: "<<g_timer.get_d("heu/save")<<")"<<std::endl;);
}

void HeuristicTable::load(const string & fpath) {
    ONLYDEV(std::cout<<"[start] load heuristics from "<<fpath<<std::endl;);
    ONLYDEV(g_timer.record_p("heu/load_start");)
    std::ifstream fin;
    fin.open(fpath,std::ios::binary|std::ios::in);

    boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;
    inbuf.push(boost::iostreams::zlib_decompressor());
    inbuf.push(fin);

    std::istream in(&inbuf);

    // load loc size
    int _loc_size;
    in.read((char *)&_loc_size,sizeof(int));

    // check loc size
    if (_loc_size!=loc_size) {
        cerr<<"the sizes of empty locations don't match!"<<endl;
        exit(-1);
    }

    // load empty locs
    int * _empty_locs=new int[loc_size];
    in.read((char *)_empty_locs,sizeof(int)*loc_size);

    // check empty locs
    for (auto i=0;i<loc_size;++i) {
        if (_empty_locs[i]!=empty_locs[i]) {
            cerr<<"the empty locations don't match!"<<endl;
            exit(-1);
        }
    }

    // load main heurisitcs
    in.read((char *)main_heuristics,sizeof(float)*loc_size*loc_size);
    
    // load sub heuristics
    if (consider_rotation)
        in.read((char *)sub_heuristics,sizeof(float)*state_size*loc_size);

    ONLYDEV(g_timer.record_d("heu/load_start","heu/load_end","heu/load");)

    ONLYDEV(std::cout<<"[end] load heuristics from "<<fpath<<". (duration: "<<g_timer.get_d("heu/load")<<")"<<std::endl;);
}

std::shared_ptr<UTIL::HeuristicTable> static_heuristic_table;
std::shared_ptr<UTIL::HeuristicTable> static_heuristic_table_no_rot;

} // namespace UTIL
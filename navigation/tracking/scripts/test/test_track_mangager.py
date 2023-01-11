from track_manager import TRACK_MANAGER

def test_cb():

    manager = TRACK_MANAGER()

    x = 1
    y = 1
    tollerance = 0.5
    n_timesteps = 10


    for i in range(n_timesteps):
        print("\n timestep", i, "\n")
        o_arr = manager.main_track.create_observations_for_one_timestep(x, y)
        print("observations: ", o_arr)
        manager.cb(o_arr)

        for track in manager.tentative_tracks:
            print("state: ", track.state_pri[:2])
            print("n: ", track.n)

    print("final estimates: ", manager.main_track.state_post)

def test_add_tentative_tracks():

    manager = TRACK_MANAGER()

    x = 0
    y = 0
    tollerance = 0.5
    n_timesteps = 15

    manager.main_track.R[0,0] = 0.001
    manager.main_track.R[1,1] = 0.001


    for i in range(n_timesteps):
        print("\n timestep", i, "\n")

        o_arr = manager.main_track.create_observations_for_one_timestep(x, y)
        print(len(manager.tentative_tracks),"tentative tracks: ")
        for track in manager.tentative_tracks:
            print("state: ", track.state_pri[:2])
            print("n: ", track.n)

        print("observations: ", o_arr)

        manager.update_status_on_tentative_tracks(o_arr)
        new_o_arr = manager.remove_o_incorporated_in_tracks(o_arr)
        manager.add_tentative_tracks(new_o_arr)

        manager.prev_observations = new_o_arr




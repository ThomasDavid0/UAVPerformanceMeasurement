local controllers = {}

controllers.TSPD = require('modules/controllers/c71_speed_controller')
controllers.TRAN = require('modules/controllers/c72_roll_angle_controller')
controllers.TPAN = require('modules/controllers/c73_pitch_angle_controller')
controllers.TPALT = require('modules/controllers/c74_pitch_alt_controller')
controllers.TYAW = require('modules/controllers/c75_yaw_controller')
controllers.TPG = require('modules/controllers/c76_pitch_g_controller')
controllers.TRCLB = require('modules/controllers/c77_turn_climb_controller')
controllers.TSPA = require('modules/controllers/c78_thr_acc_controller')


function controllers.reset_all(defaults)
    controllers.TSPD:reset(defaults.TSPD)
    controllers.TRAN:reset(defaults.TRAN)
    controllers.TPAN:reset(defaults.TPAN)
    controllers.TPALT:reset(defaults.TPALT)
    controllers.TYAW:reset(defaults.TYAW)
    controllers.TPG:reset(defaults.TPG)
    controllers.TRCLB:reset(defaults.TRCLB)
    controllers.TSPA:reset(defaults.TSPA)
end

return controllers
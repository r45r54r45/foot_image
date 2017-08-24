var express = require('express');
var router = express.Router();
var multer = require('multer');
var path = require('path')
var fs = require('fs');
var storage = multer.diskStorage({
    destination: function (req, file, cb) {
        cb(null, 'uploaded');
    },

    // 서버에 저장할 파일 명
    filename: function (req, file, cb) {
        file.uploadedFile = {
            name: req.params.filename,
            ext: file.mimetype.split('/')[1]
        };
        cb(null, new Date().getTime() + '.' + file.uploadedFile.ext);
    }
})
/* GET home page. */
router.get('/', function (req, res, next) {
    // 이미지 리스트 역순으로 보내주기
    fs.readdir('/Users/woohyunkim/Documents/SourceFolder/opencv/opencv/opencv/server/uploaded', function (err, files) {
        files = files.map((item) => {
            return {
                url: item,
                newName: ''
            }
        }).reverse();
        res.json({
            files
        });
    })
});
router.get('/image/:id', function (req, res, next) {
    // 이미지 리스트 역순으로 보내주기
    const exec = require('child_process').exec;
    exec(`./MainProgram ../server/uploaded/${req.params.id}`, {
        // cwd: '/usr/src/app'
        maxBuffer: 1024 * 1024 * 5,
        cwd: '/Users/woohyunkim/Documents/SourceFolder/opencv/opencv/opencv/cmake-build-debug'
    }, function (err, stdout, stderr) {
        if (err) {
            console.log("node couldn't execute the command" + err);
            // node couldn't execute the command
            return;
        }
        // the *entire* stdout and stderr (buffered)
        let result;
        try {
            result = JSON.parse(stdout);
            if (result.error) {
                console.log("error: ", result.error)
            }
        } catch (e) {
            result = stdout;
            console.log("stdout: ", result)
        }

        console.log(`stderr: ${stderr}`);


        res.send("good");
    });
});
router.get('/delete/:id', function (req, res, next) {
    console.log(path.join(__dirname, '../uploaded/' + req.params.id))
    fs.unlinkSync(path.join(__dirname, '../uploaded/' + req.params.id));
    res.json({
        result: true
    })
})
router.get('/rename/:id/:newName', function (req, res, next) {
    console.log(path.join(__dirname, '../uploaded/' + req.params.id))
    fs.rename(path.join(__dirname, '../uploaded/' + req.params.id), path.join(__dirname, '../uploaded/' + req.params.newName + "." + req.params.id.split(".")[1]), (err) => {
        if (err) throw err;
        console.log('renamed complete');
        res.json({
            result: true
        })
    });

})
router.post('/', multer({storage: storage}).single('file'), function (req, res, next) {
    const exec = require('child_process').exec;
    console.log("image receiving")
    exec(`./MainProgram ../server/${req.file.path}`, {
        // cwd: '/usr/src/app'
        cwd: '/Users/woohyunkim/Documents/SourceFolder/opencv/opencv/opencv/cmake-build-debug'
    }, function (err, stdout, stderr) {
        if (err) {
            console.log("node couldn't execute the command" + err);
            // node couldn't execute the command
            return;
        }
        // the *entire* stdout and stderr (buffered)
        console.log(`stdout: ${stdout}`);
        console.log(`stderr: ${stderr}`);
        res.json(JSON.parse(stdout));
    });

})


module.exports = router;

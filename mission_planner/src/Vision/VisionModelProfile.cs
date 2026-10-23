// SPDX-License-Identifier: Apache-2.0
using System;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text.RegularExpressions;
using System.Threading;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    [JsonObject(MemberSerialization.OptIn)]
    internal sealed class VisionModelProfile
    {
        internal const string BuiltinHash = "c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d";
        [JsonProperty(Required = Required.Always)] public int SchemaVersion { get; set; }
        [JsonProperty(Required = Required.Always)] public string Name { get; set; }
        [JsonProperty(Required = Required.Always)] public string Model { get; set; }
        [JsonProperty(Required = Required.Always)] public string Sha256 { get; set; }
        [JsonProperty(Required = Required.Always)] public int InputSize { get; set; }
        [JsonProperty(Required = Required.Always)] public string Layout { get; set; }
        [JsonProperty(Required = Required.Always)] public string ColorOrder { get; set; }
        [JsonProperty(Required = Required.Always)] public float Scale { get; set; }
        [JsonProperty(Required = Required.Always)] public int Padding { get; set; }
        [JsonProperty(Required = Required.Always)] public string Letterbox { get; set; }
        [JsonProperty(Required = Required.Always)] public string Output { get; set; }
        [JsonProperty(Required = Required.Always)] public int[] Strides { get; set; }
        [JsonProperty(Required = Required.Always)] public string[] Labels { get; set; }
        [JsonProperty(Required = Required.Always)] public int[] AllowedClasses { get; set; }
        [JsonProperty(Required = Required.Always)] public float ScoreThreshold { get; set; }
        [JsonProperty(Required = Required.Always)] public float NmsThreshold { get; set; }

        internal string Identity => Name + " / " + Sha256.Substring(0, 12);
        internal int Rows => Strides.Sum(stride => (InputSize / stride) * (InputSize / stride));

        internal static VisionModelProfile Builtin()
        {
            return new VisionModelProfile
            {
                SchemaVersion = 1, Name = "YOLOX Nano COCO vehicles", Model = "yolox_nano.onnx",
                Sha256 = BuiltinHash, InputSize = 416, Layout = "NCHW", ColorOrder = "BGR", Scale = 1,
                Padding = 114, Letterbox = "top-left", Output = "yolox-grid", Strides = new[] { 8, 16, 32 },
                AllowedClasses = new[] { 1, 2, 3, 5, 7 }, ScoreThreshold = .35f, NmsThreshold = .45f,
                Labels = ("person,bicycle,car,motorcycle,airplane,bus,train,truck,boat,traffic light," +
                    "fire hydrant,stop sign,parking meter,bench,bird,cat,dog,horse,sheep,cow,elephant,bear," +
                    "zebra,giraffe,backpack,umbrella,handbag,tie,suitcase,frisbee,skis,snowboard,sports ball," +
                    "kite,baseball bat,baseball glove,skateboard,surfboard,tennis racket,bottle,wine glass," +
                    "cup,fork,knife,spoon,bowl,banana,apple,sandwich,orange,broccoli,carrot,hot dog,pizza," +
                    "donut,cake,chair,couch,potted plant,bed,dining table,toilet,tv,laptop,mouse,remote," +
                    "keyboard,cell phone,microwave,oven,toaster,sink,refrigerator,book,clock,vase,scissors," +
                    "teddy bear,hair drier,toothbrush").Split(',')
            };
        }

        internal static VisionModelProfile Load(string path)
        {
            var info = new FileInfo(path);
            if (!info.Exists || info.Length > 65536) { throw new FormatException("Invalid vision profile file."); }
            var document = ReadDocument(File.ReadAllText(path));
            var profile = document.ToObject<VisionModelProfile>(JsonSerializer.Create(
                new JsonSerializerSettings { MissingMemberHandling = MissingMemberHandling.Error, MaxDepth = 12 }));
            if (profile == null) { throw new FormatException("Vision profile is empty."); }
            profile.Validate();
            profile.Model = Path.GetFullPath(Path.Combine(info.DirectoryName, profile.Model));
            return profile;
        }

        private static JObject ReadDocument(string json)
        {
            using (var reader = new JsonTextReader(new StringReader(json))
                { MaxDepth = 12, DateParseHandling = DateParseHandling.None })
            {
                var document = JToken.Load(reader, new JsonLoadSettings
                    { DuplicatePropertyNameHandling = DuplicatePropertyNameHandling.Error }) as JObject;
                if (document == null) { throw new JsonSerializationException("Expected a vision profile object."); }
                while (reader.Read())
                {
                    if (reader.TokenType != JsonToken.Comment)
                    { throw new JsonSerializationException("Unexpected content after vision profile."); }
                }
                foreach (var field in document.Properties())
                {
                    var type = typeof(VisionModelProfile).GetProperty(field.Name)?.PropertyType;
                    var value = field.Value.Type;
                    bool valid = type == typeof(string) ? value == JTokenType.String :
                        type == typeof(int) ? value == JTokenType.Integer :
                        type == typeof(float) ? value == JTokenType.Float || value == JTokenType.Integer :
                        type == typeof(int[]) ? field.Value is JArray integers &&
                            integers.All(item => item.Type == JTokenType.Integer) :
                        type == typeof(string[]) && field.Value is JArray strings &&
                            strings.All(item => item.Type == JTokenType.String);
                    if (!valid) { throw new JsonSerializationException("Invalid vision profile field: " + field.Name); }
                }
                return document;
            }
        }

        internal void Validate()
        {
            if (SchemaVersion != 1 || string.IsNullOrWhiteSpace(Name) || Name.Length > 80 ||
                Name != Name.Trim() || Name.Any(char.IsControl) || string.IsNullOrWhiteSpace(Model) ||
                Sha256 == null || !Regex.IsMatch(Sha256, "\\A[0-9a-f]{64}\\z") || InputSize < 160 ||
                InputSize > 1280 || InputSize % 32 != 0)
            { throw new FormatException("Invalid vision model identity, hash or input size."); }
            if (Layout != "NCHW" || ColorOrder != "BGR" || Scale != 1 || Padding != 114 ||
                Letterbox != "top-left" || Output != "yolox-grid" || Strides == null ||
                !Strides.SequenceEqual(new[] { 8, 16, 32 }))
            { throw new FormatException("Vision requires raw YOLOX grid output and BGR NCHW 0..255 input."); }
            if (Labels == null || Labels.Length < 1 || Labels.Length > 256 ||
                Labels.Any(label => string.IsNullOrWhiteSpace(label) || label.Length > 80 ||
                    label.Any(char.IsControl)) ||
                AllowedClasses == null || AllowedClasses.Length < 1 ||
                AllowedClasses.Any(id => id < 0 || id >= Labels.Length) ||
                AllowedClasses.Distinct().Count() != AllowedClasses.Length ||
                !VideoDetection.Finite(ScoreThreshold) || ScoreThreshold <= 0 || ScoreThreshold > 1 ||
                !VideoDetection.Finite(NmsThreshold) || NmsThreshold <= 0 || NmsThreshold >= 1)
            { throw new FormatException("Invalid vision labels, allowed classes or score thresholds."); }
        }

        internal byte[] ReadModel(CancellationToken token)
        {
            using (var stream = new FileStream(Model, FileMode.Open, FileAccess.Read, FileShare.Read))
            {
                if (stream.Length < 1 || stream.Length > 256 * 1024 * 1024)
                { throw new FormatException("Vision model must be from 1 byte to 256 MiB."); }
                var bytes = new byte[(int)stream.Length];
                int offset = 0;
                while (offset < bytes.Length)
                {
                    token.ThrowIfCancellationRequested();
                    int count = stream.Read(bytes, offset, Math.Min(65536, bytes.Length - offset));
                    if (count == 0) { throw new IOException("Vision model was truncated."); }
                    offset += count;
                }
                if (Hash(bytes) != Sha256)
                { throw new FormatException("Vision model SHA256 does not match its profile."); }
                token.ThrowIfCancellationRequested();
                return bytes;
            }
        }

        internal static string Hash(byte[] bytes)
        {
            using (var hash = SHA256.Create())
            { return BitConverter.ToString(hash.ComputeHash(bytes)).Replace("-", "").ToLowerInvariant(); }
        }
    }
}

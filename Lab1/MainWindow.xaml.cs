using GraphicsLib;
using GraphicsLib.Shaders;
using GraphicsLib.Types;
using Microsoft.Win32;
using System.ComponentModel;
using System.Diagnostics;
using System.Numerics;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace Lab1
{
    public partial class MainWindow : Window
    {
        private readonly OpenFileDialog ofd;
        private static Obj? obj;
        private static Camera camera;
        private static Scene scene;
        private Renderer renderer;
        private Point oldPos;

        // Shadow and light position tracking
        private bool shadowsEnabled = false;
        private bool shadowsRendered = false;
        private Vector3 lastLightPosition;
        // Трекинг изменений трансформации объекта
        private bool transformationChanged = false;
        static MainWindow()
        {
            camera = new Camera();
            scene = new Scene(camera);
        }

        public MainWindow()
        {
            InitializeComponent();

            ofd = new OpenFileDialog
            {
                CheckFileExists = true,
                CheckPathExists = true,
                Multiselect = false,
                ValidateNames = true,
                Filter = "GLTF files(*.gltf)|*.gltf|OBJ files (*.obj)|*.obj|All files (*.*)|*.*"
            };
            ofd.FileOk += OnFileOpened;
            Height = SystemParameters.PrimaryScreenHeight / 1.25;
            Width = SystemParameters.PrimaryScreenWidth / 1.25;

            renderer = new Renderer(scene);

            lastLightPosition = new Vector3(-1000f, 100f, 1000f);
            scene.LightPosition = lastLightPosition;

            // Bind UI events
            lightX.TextChanged += (s, e) => UpdateLightPosition();
            lightY.TextChanged += (s, e) => UpdateLightPosition();
            lightZ.TextChanged += (s, e) => UpdateLightPosition();
            useCameraPosition.Checked += (s, e) => UpdateLightPosition();
            useCameraPosition.Unchecked += (s, e) => UpdateLightPosition();
            enableShadows.Checked += EnableShadows_Checked;
            enableShadows.Unchecked += EnableShadows_Unchecked;
        }

        private void OnFileOpened(object? sender, CancelEventArgs e)
        {
            fileTextBox.Text = ofd.FileName;

            if (System.IO.Path.GetExtension(ofd.FileName).Equals(".obj"))
                obj = Parser.ParseObjFile(ofd.FileName);
            else
                obj = Parser.ParseGltfFile(ofd.FileName);
            obj.transformation.Reset();
            shadowsRendered = false; // Reset shadows on new object load
            scene.Obj = obj;
            renderer.ClearTriangleCache();
            Draw();
        }

        private void UpdateLightPosition()
        {
            if (useCameraPosition.IsChecked == true)
            {
                scene.LightPosition = camera.Position;
            }
            else
            {
                try
                {
                    float x = float.Parse(lightX.Text);
                    float y = float.Parse(lightY.Text);
                    float z = float.Parse(lightZ.Text);
                    scene.LightPosition = new Vector3(x, y, z);
                }
                catch
                {
                    // Revert to last valid position if parsing fails
                    scene.LightPosition = lastLightPosition;
                    lightX.Text = lastLightPosition.X.ToString();
                    lightY.Text = lastLightPosition.Y.ToString();
                    lightZ.Text = lastLightPosition.Z.ToString();
                }
            }
            // Обновляем матрицы света, если тени включены
            if (shadowsEnabled)
            {
                // Направленный свет смотрит на начало координат (можно настроить)
                scene.LightViewMatrix = Matrix4x4.CreateLookAt(scene.LightPosition, Vector3.Zero, Vector3.UnitY);
                // Ортографическая проекция (размеры зависят от сцены, настройте под свои нужды)
                scene.LightProjectionMatrix = Matrix4x4.CreateOrthographic(20f, 20f, 0.1f, 100f);
            }
            // Check if light position changed
            if (scene.LightPosition != lastLightPosition && shadowsEnabled)
            {
                lastLightPosition = scene.LightPosition;
                shadowsRendered = false; // Trigger shadow re-rendering
            }
            Draw();
        }

        private void EnableShadows_Checked(object sender, RoutedEventArgs e)
        {
            shadowsEnabled = true;
            shadowsRendered = false; // Force shadow rendering
            Draw();
        }

        private void EnableShadows_Unchecked(object sender, RoutedEventArgs e)
        {
            shadowsEnabled = false;
            Draw();
        }

        private void Draw()
        {
            if (renderer == null)
                return;
            try
            {
                WriteableBitmap bitmap = new WriteableBitmap(
                    ((int)canvas.ActualWidth), ((int)canvas.ActualHeight), 96, 96, PixelFormats.Bgra32, null);
                renderer.Bitmap = bitmap;
                int FaceCount = 0;
                if (obj != null)
                {
                    if (shadowsEnabled && !shadowsRendered)
                    {
                        shadowsRendered = true;
                    }
                    // Очистка кэша, если трансформация изменилась
                    if (transformationChanged)
                    {
                        renderer.ClearTriangleCache();
                        transformationChanged = false;
                    }
                    if (renderMode == "Полигоны")
                        FaceCount = renderer.RenderSolid();
                    else if (renderMode == "Smooth")
                    {
                        renderer.shadowsEnabled = shadowsEnabled;
                        FaceCount = renderer.Render<PhongShader, PhongShader.Vertex>();
                    }
                    else if (renderMode == "Textured")
                    {
                        renderer.shadowsEnabled = shadowsEnabled;
                        FaceCount = renderer.Render<PhongTexturedShader, PhongTexturedShader.Vertex>();
                    }
                    else
                        FaceCount = renderer.RenderCarcass();
                    FlatCount.Text = string.Join(' ', Resources["FlatCountString"].ToString(), FaceCount.ToString());
                }
                bitmap.Lock();
                bitmap.AddDirtyRect(new Int32Rect(0, 0, bitmap.PixelWidth, bitmap.PixelHeight));
                bitmap.Unlock();
                canvas.Child = new Image { Source = bitmap };
                renderer.Bitmap = null;
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private void ButtonOpenFile_Click(object sender, RoutedEventArgs e)
        {
            ofd.ShowDialog();
        }

        private void Window_MouseDown(object sender, MouseButtonEventArgs e)
        {
            Mouse.Capture(canvas);
            oldPos = Mouse.GetPosition(canvas);
        }

        private void Window_MouseMove(object sender, MouseEventArgs e)
        {
            if (e.LeftButton == MouseButtonState.Pressed && Mouse.Captured == canvas && oldPos.X != -1)
            {
                Point newPos = Mouse.GetPosition(canvas);
                float dx = (float)(newPos.X - oldPos.X);
                float dy = (float)(newPos.Y - oldPos.Y);
                if ((Keyboard.Modifiers & ModifierKeys.Shift) != 0)
                {
                    // Handle shift + mouse move if needed
                }
                else
                {
                    camera.RotateAroundTargetHorizontal((float)(-dx * MathF.PI / canvas.ActualWidth));
                    camera.RotateAroundTargetVertical((float)(-dy * MathF.PI / canvas.ActualHeight));
                }
                Draw();
                oldPos = newPos;
            }
            else
            {
                oldPos = new Point(-1, -1);
            }
        }

        private void Window_MouseUp(object sender, MouseButtonEventArgs e)
        {
            Mouse.Capture(canvas, CaptureMode.None);
            oldPos = new Point(-1, -1);
        }

        private void Window_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            float dz = (float)e.Delta;
            float step = Keyboard.Modifiers == ModifierKeys.Control ? 0.002f : 0.0005f;
            camera.MoveTowardTarget(dz * step * camera.Distance);
            Draw();
        }

        private void Window_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            Draw();
        }

        private void canvas_GotMouseCapture(object sender, MouseEventArgs e) { }

        private void canvas_LostMouseCapture(object sender, MouseEventArgs e) { }

        private string renderMode = "Полигоны";
        private string TransformMode = "Двигать";

        private void RadioButton_Checked(object sender, RoutedEventArgs e)
        {
            if (sender is RadioButton radioButton && radioButton.IsChecked == true)
            {
                switch (radioButton.GroupName)
                {
                    case "TransformMode":
                        TransformMode = ((RadioButton)sender).Content.ToString()!;
                        break;
                    case "RenderMode":
                        renderMode = ((RadioButton)sender).Content.ToString()!;
                        Draw();
                        break;
                }
            }
        }

        private static float speed = 0.5f;
        private Dictionary<Key, Action> moveActions = new() {
            { Key.OemPlus, MakeLarger },
            { Key.OemMinus, MakeSmaller },
            { Key.W, () => {
                if ((Keyboard.Modifiers & ModifierKeys.Shift) != 0)
                    obj!.transformation.Offset.Z += speed;
                else
                    obj!.transformation.Offset.Y += speed;
            } },
            { Key.S, () => {
                if ((Keyboard.Modifiers & ModifierKeys.Shift) != 0)
                    obj!.transformation.Offset.Z -= speed;
                else
                    obj!.transformation.Offset.Y -= speed;
            } },
            { Key.D, () => { obj!.transformation.Offset.X += speed; } },
            { Key.A, () => { obj!.transformation.Offset.X -= speed; } }
        };

        private Dictionary<Key, Action> rotateActions = new() {
            { Key.OemPlus, MakeLarger },
            { Key.OemMinus, MakeSmaller },
            { Key.W, () => { obj!.transformation.AngleX += speed; } },
            { Key.S, () => { obj!.transformation.AngleX -= speed; } },
            { Key.A, () => {
                if ((Keyboard.Modifiers & ModifierKeys.Shift) != 0)
                    obj!.transformation.AngleY += speed;
                else
                    obj!.transformation.AngleZ += speed;
            } },
            { Key.D, () => {
                if ((Keyboard.Modifiers & ModifierKeys.Shift) != 0)
                    obj!.transformation.AngleY -= speed;
                else
                    obj!.transformation.AngleZ -= speed;
            } }
        };

        private static void MakeLarger()
        {
            obj!.transformation.Scale += speed / 10.0f;
        }

        private static void MakeSmaller()
        {
            obj!.transformation.Scale -= speed / 10.0f;
        }

        private void canvas_KeyDown(object sender, KeyEventArgs e)
        {
            if (obj == null) e.Handled = true;
            else
            {
                Dictionary<Key, Action> handlers = moveActions;
                if (TransformMode == "Поворачивать")
                {
                    handlers = rotateActions;
                }

                if (handlers.TryGetValue(e.Key, out Action? action))
                {
                    speed = (Keyboard.Modifiers & ModifierKeys.Control) != 0 ? 1f : 0.25f;
                    if (TransformMode == "Двигать")
                    {
                        speed *= camera.Distance / 500;
                    }
                    action();
                    transformationChanged = true; // Отмечаем изменение трансформации
                    Draw();
                }
            }
        }
    }
}